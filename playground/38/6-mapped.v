// Benchmark "adder" written by ABC on Thu Jul 18 07:27:08 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n307, new_n309, new_n310,
    new_n313, new_n314, new_n315, new_n317, new_n319;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  tech160nm_fiaoi012aa1n05x5   g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nor042aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand42aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor002aa1n10x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  oai012aa1n02x5               g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  oai012aa1n12x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  tech160nm_fixnrc02aa1n04x5   g014(.a(\b[7] ), .b(\a[8] ), .out0(new_n110));
  tech160nm_fixnrc02aa1n03p5x5 g015(.a(\b[6] ), .b(\a[7] ), .out0(new_n111));
  nor022aa1n08x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nand42aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nand42aa1n03x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nor022aa1n04x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nona23aa1n03x5               g020(.a(new_n114), .b(new_n113), .c(new_n115), .d(new_n112), .out0(new_n116));
  nor043aa1n06x5               g021(.a(new_n116), .b(new_n111), .c(new_n110), .o1(new_n117));
  tech160nm_fioai012aa1n04x5   g022(.a(new_n114), .b(new_n115), .c(new_n112), .o1(new_n118));
  nor042aa1n09x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(new_n119), .o1(new_n120));
  oao003aa1n02x5               g025(.a(\a[8] ), .b(\b[7] ), .c(new_n120), .carry(new_n121));
  oai013aa1n03x5               g026(.a(new_n121), .b(new_n111), .c(new_n110), .d(new_n118), .o1(new_n122));
  tech160nm_fixorc02aa1n03p5x5 g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n124));
  tech160nm_fixorc02aa1n03p5x5 g029(.a(\a[10] ), .b(\b[9] ), .out0(new_n125));
  xnbna2aa1n03x5               g030(.a(new_n125), .b(new_n124), .c(new_n98), .out0(\s[10] ));
  nand42aa1n02x5               g031(.a(new_n109), .b(new_n117), .o1(new_n127));
  norp03aa1n02x5               g032(.a(new_n110), .b(new_n111), .c(new_n118), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n121), .b(new_n128), .out0(new_n129));
  nanp02aa1n02x5               g034(.a(new_n125), .b(new_n123), .o1(new_n130));
  norp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  tech160nm_fioai012aa1n05x5   g037(.a(new_n132), .b(new_n131), .c(new_n97), .o1(new_n133));
  aoai13aa1n03x5               g038(.a(new_n133), .b(new_n130), .c(new_n127), .d(new_n129), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d24x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  nand42aa1n02x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n136), .out0(new_n139));
  nanp02aa1n03x5               g044(.a(new_n134), .b(new_n139), .o1(new_n140));
  norp02aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand22aa1n03x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n140), .c(new_n137), .out0(\s[12] ));
  nona23aa1n03x5               g049(.a(new_n138), .b(new_n142), .c(new_n141), .d(new_n136), .out0(new_n145));
  nano22aa1n02x4               g050(.a(new_n145), .b(new_n123), .c(new_n125), .out0(new_n146));
  aoai13aa1n03x5               g051(.a(new_n146), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n147));
  oai012aa1n02x5               g052(.a(new_n142), .b(new_n141), .c(new_n136), .o1(new_n148));
  tech160nm_fioai012aa1n04x5   g053(.a(new_n148), .b(new_n145), .c(new_n133), .o1(new_n149));
  inv000aa1n06x5               g054(.a(new_n149), .o1(new_n150));
  nand42aa1d28x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nor002aa1d32x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  norb02aa1n02x5               g057(.a(new_n151), .b(new_n152), .out0(new_n153));
  xnbna2aa1n03x5               g058(.a(new_n153), .b(new_n147), .c(new_n150), .out0(\s[13] ));
  orn002aa1n02x5               g059(.a(\a[13] ), .b(\b[12] ), .o(new_n155));
  nand22aa1n03x5               g060(.a(new_n127), .b(new_n129), .o1(new_n156));
  aoai13aa1n02x5               g061(.a(new_n153), .b(new_n149), .c(new_n156), .d(new_n146), .o1(new_n157));
  nor042aa1n06x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nanp02aa1n24x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  xnbna2aa1n03x5               g065(.a(new_n160), .b(new_n157), .c(new_n155), .out0(\s[14] ));
  oai012aa1n03x5               g066(.a(new_n159), .b(new_n158), .c(new_n152), .o1(new_n162));
  nano23aa1d15x5               g067(.a(new_n158), .b(new_n152), .c(new_n159), .d(new_n151), .out0(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  aoai13aa1n06x5               g069(.a(new_n162), .b(new_n164), .c(new_n147), .d(new_n150), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nand02aa1n03x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nor042aa1n06x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanb02aa1n06x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  norb02aa1n02x5               g074(.a(new_n165), .b(new_n169), .out0(new_n170));
  nor002aa1n06x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanp02aa1n04x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nanb02aa1n06x5               g077(.a(new_n171), .b(new_n172), .out0(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n168), .c(new_n165), .d(new_n167), .o1(new_n174));
  nona22aa1n02x4               g079(.a(new_n172), .b(new_n171), .c(new_n168), .out0(new_n175));
  oai012aa1n02x5               g080(.a(new_n174), .b(new_n170), .c(new_n175), .o1(\s[16] ));
  nano23aa1n03x7               g081(.a(new_n141), .b(new_n136), .c(new_n142), .d(new_n138), .out0(new_n177));
  nona22aa1d24x5               g082(.a(new_n163), .b(new_n173), .c(new_n169), .out0(new_n178));
  nano32aa1d12x5               g083(.a(new_n178), .b(new_n177), .c(new_n125), .d(new_n123), .out0(new_n179));
  aoai13aa1n12x5               g084(.a(new_n179), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n180));
  oai012aa1n02x5               g085(.a(new_n172), .b(new_n171), .c(new_n168), .o1(new_n181));
  oai013aa1n02x4               g086(.a(new_n181), .b(new_n162), .c(new_n169), .d(new_n173), .o1(new_n182));
  aoib12aa1n09x5               g087(.a(new_n182), .b(new_n149), .c(new_n178), .out0(new_n183));
  tech160nm_fixorc02aa1n03p5x5 g088(.a(\a[17] ), .b(\b[16] ), .out0(new_n184));
  xnbna2aa1n03x5               g089(.a(new_n184), .b(new_n180), .c(new_n183), .out0(\s[17] ));
  inv000aa1d42x5               g090(.a(\a[17] ), .o1(new_n186));
  nanb02aa1d24x5               g091(.a(\b[16] ), .b(new_n186), .out0(new_n187));
  oabi12aa1n06x5               g092(.a(new_n182), .b(new_n150), .c(new_n178), .out0(new_n188));
  aoai13aa1n02x5               g093(.a(new_n184), .b(new_n188), .c(new_n156), .d(new_n179), .o1(new_n189));
  xnrc02aa1n02x5               g094(.a(\b[17] ), .b(\a[18] ), .out0(new_n190));
  xobna2aa1n03x5               g095(.a(new_n190), .b(new_n189), .c(new_n187), .out0(\s[18] ));
  inv020aa1n04x5               g096(.a(\a[18] ), .o1(new_n192));
  xroi22aa1d04x5               g097(.a(new_n186), .b(\b[16] ), .c(new_n192), .d(\b[17] ), .out0(new_n193));
  inv000aa1d42x5               g098(.a(new_n193), .o1(new_n194));
  oaoi03aa1n12x5               g099(.a(\a[18] ), .b(\b[17] ), .c(new_n187), .o1(new_n195));
  inv000aa1d42x5               g100(.a(new_n195), .o1(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n194), .c(new_n180), .d(new_n183), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n04x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nor002aa1n06x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nor042aa1n04x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nand02aa1n04x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  aoai13aa1n03x5               g109(.a(new_n204), .b(new_n201), .c(new_n197), .d(new_n200), .o1(new_n205));
  aoai13aa1n02x5               g110(.a(new_n193), .b(new_n188), .c(new_n156), .d(new_n179), .o1(new_n206));
  nanb02aa1n02x5               g111(.a(new_n201), .b(new_n200), .out0(new_n207));
  norb03aa1n02x5               g112(.a(new_n203), .b(new_n201), .c(new_n202), .out0(new_n208));
  aoai13aa1n02x7               g113(.a(new_n208), .b(new_n207), .c(new_n206), .d(new_n196), .o1(new_n209));
  nanp02aa1n03x5               g114(.a(new_n205), .b(new_n209), .o1(\s[20] ));
  nano23aa1d15x5               g115(.a(new_n202), .b(new_n201), .c(new_n203), .d(new_n200), .out0(new_n211));
  nanb03aa1n09x5               g116(.a(new_n190), .b(new_n211), .c(new_n184), .out0(new_n212));
  tech160nm_fioai012aa1n02p5x5 g117(.a(new_n203), .b(new_n202), .c(new_n201), .o1(new_n213));
  aobi12aa1n18x5               g118(.a(new_n213), .b(new_n211), .c(new_n195), .out0(new_n214));
  aoai13aa1n04x5               g119(.a(new_n214), .b(new_n212), .c(new_n180), .d(new_n183), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nand22aa1n09x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  norp02aa1n24x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  aoi012aa1n03x5               g123(.a(new_n218), .b(new_n215), .c(new_n217), .o1(new_n219));
  nor042aa1n09x5               g124(.a(\b[21] ), .b(\a[22] ), .o1(new_n220));
  nand02aa1d04x5               g125(.a(\b[21] ), .b(\a[22] ), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  norb02aa1n02x5               g127(.a(new_n217), .b(new_n218), .out0(new_n223));
  norb03aa1n02x5               g128(.a(new_n221), .b(new_n218), .c(new_n220), .out0(new_n224));
  aob012aa1n03x5               g129(.a(new_n224), .b(new_n215), .c(new_n223), .out0(new_n225));
  oaih12aa1n02x5               g130(.a(new_n225), .b(new_n219), .c(new_n222), .o1(\s[22] ));
  nano23aa1n03x7               g131(.a(new_n220), .b(new_n218), .c(new_n221), .d(new_n217), .out0(new_n227));
  nanp03aa1n02x5               g132(.a(new_n193), .b(new_n211), .c(new_n227), .o1(new_n228));
  inv020aa1n02x5               g133(.a(new_n214), .o1(new_n229));
  and002aa1n02x5               g134(.a(\b[21] ), .b(\a[22] ), .o(new_n230));
  oab012aa1n02x4               g135(.a(new_n230), .b(new_n218), .c(new_n220), .out0(new_n231));
  aoi012aa1n02x5               g136(.a(new_n231), .b(new_n229), .c(new_n227), .o1(new_n232));
  aoai13aa1n06x5               g137(.a(new_n232), .b(new_n228), .c(new_n180), .d(new_n183), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1d32x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  nand42aa1n20x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(new_n237));
  nor022aa1n08x5               g142(.a(\b[23] ), .b(\a[24] ), .o1(new_n238));
  nand42aa1d28x5               g143(.a(\b[23] ), .b(\a[24] ), .o1(new_n239));
  nanb02aa1n02x5               g144(.a(new_n238), .b(new_n239), .out0(new_n240));
  aoai13aa1n03x5               g145(.a(new_n240), .b(new_n235), .c(new_n233), .d(new_n237), .o1(new_n241));
  nona22aa1d18x5               g146(.a(new_n239), .b(new_n238), .c(new_n235), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  aob012aa1n03x5               g148(.a(new_n243), .b(new_n233), .c(new_n237), .out0(new_n244));
  nanp02aa1n03x5               g149(.a(new_n241), .b(new_n244), .o1(\s[24] ));
  nano23aa1n02x5               g150(.a(new_n235), .b(new_n238), .c(new_n239), .d(new_n236), .out0(new_n246));
  nand02aa1n02x5               g151(.a(new_n246), .b(new_n227), .o1(new_n247));
  nanb03aa1n12x5               g152(.a(new_n247), .b(new_n193), .c(new_n211), .out0(new_n248));
  nand22aa1n03x5               g153(.a(new_n211), .b(new_n195), .o1(new_n249));
  aoi022aa1n06x5               g154(.a(new_n246), .b(new_n231), .c(new_n239), .d(new_n242), .o1(new_n250));
  aoai13aa1n06x5               g155(.a(new_n250), .b(new_n247), .c(new_n249), .d(new_n213), .o1(new_n251));
  inv000aa1n02x5               g156(.a(new_n251), .o1(new_n252));
  aoai13aa1n04x5               g157(.a(new_n252), .b(new_n248), .c(new_n180), .d(new_n183), .o1(new_n253));
  xorb03aa1n02x5               g158(.a(new_n253), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g159(.a(\b[24] ), .b(\a[25] ), .o1(new_n255));
  xorc02aa1n02x5               g160(.a(\a[25] ), .b(\b[24] ), .out0(new_n256));
  xnrc02aa1n02x5               g161(.a(\b[25] ), .b(\a[26] ), .out0(new_n257));
  aoai13aa1n03x5               g162(.a(new_n257), .b(new_n255), .c(new_n253), .d(new_n256), .o1(new_n258));
  oabi12aa1n12x5               g163(.a(new_n257), .b(\a[25] ), .c(\b[24] ), .out0(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  aob012aa1n03x5               g165(.a(new_n260), .b(new_n253), .c(new_n256), .out0(new_n261));
  nanp02aa1n03x5               g166(.a(new_n258), .b(new_n261), .o1(\s[26] ));
  norb02aa1n02x7               g167(.a(new_n256), .b(new_n257), .out0(new_n263));
  norb03aa1n03x5               g168(.a(new_n263), .b(new_n212), .c(new_n247), .out0(new_n264));
  inv000aa1n02x5               g169(.a(new_n264), .o1(new_n265));
  nanp02aa1n02x5               g170(.a(\b[25] ), .b(\a[26] ), .o1(new_n266));
  aoi022aa1n12x5               g171(.a(new_n251), .b(new_n263), .c(new_n266), .d(new_n259), .o1(new_n267));
  aoai13aa1n09x5               g172(.a(new_n267), .b(new_n265), .c(new_n180), .d(new_n183), .o1(new_n268));
  xorb03aa1n03x5               g173(.a(new_n268), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  xorc02aa1n12x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  xnrc02aa1n12x5               g176(.a(\b[27] ), .b(\a[28] ), .out0(new_n272));
  aoai13aa1n03x5               g177(.a(new_n272), .b(new_n270), .c(new_n268), .d(new_n271), .o1(new_n273));
  aoai13aa1n06x5               g178(.a(new_n264), .b(new_n188), .c(new_n156), .d(new_n179), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n271), .o1(new_n275));
  norp02aa1n02x5               g180(.a(new_n272), .b(new_n270), .o1(new_n276));
  aoai13aa1n02x5               g181(.a(new_n276), .b(new_n275), .c(new_n274), .d(new_n267), .o1(new_n277));
  nanp02aa1n03x5               g182(.a(new_n273), .b(new_n277), .o1(\s[28] ));
  norb02aa1d21x5               g183(.a(new_n271), .b(new_n272), .out0(new_n279));
  inv000aa1d42x5               g184(.a(new_n279), .o1(new_n280));
  xorc02aa1n12x5               g185(.a(\a[29] ), .b(\b[28] ), .out0(new_n281));
  inv000aa1d42x5               g186(.a(new_n281), .o1(new_n282));
  tech160nm_fiaoi012aa1n05x5   g187(.a(new_n276), .b(\a[28] ), .c(\b[27] ), .o1(new_n283));
  norp02aa1n02x5               g188(.a(new_n283), .b(new_n282), .o1(new_n284));
  aoai13aa1n02x5               g189(.a(new_n284), .b(new_n280), .c(new_n274), .d(new_n267), .o1(new_n285));
  aoai13aa1n03x5               g190(.a(new_n282), .b(new_n283), .c(new_n268), .d(new_n279), .o1(new_n286));
  nanp02aa1n03x5               g191(.a(new_n286), .b(new_n285), .o1(\s[29] ));
  xorb03aa1n02x5               g192(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g193(.a(new_n272), .b(new_n271), .c(new_n281), .out0(new_n289));
  nanp02aa1n03x5               g194(.a(new_n268), .b(new_n289), .o1(new_n290));
  norp02aa1n02x5               g195(.a(\b[28] ), .b(\a[29] ), .o1(new_n291));
  aoi012aa1n02x5               g196(.a(new_n291), .b(new_n283), .c(new_n281), .o1(new_n292));
  xorc02aa1n02x5               g197(.a(\a[30] ), .b(\b[29] ), .out0(new_n293));
  inv000aa1n02x5               g198(.a(new_n289), .o1(new_n294));
  oai012aa1n02x5               g199(.a(new_n293), .b(\b[28] ), .c(\a[29] ), .o1(new_n295));
  aoi012aa1n02x5               g200(.a(new_n295), .b(new_n283), .c(new_n281), .o1(new_n296));
  aoai13aa1n02x7               g201(.a(new_n296), .b(new_n294), .c(new_n274), .d(new_n267), .o1(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n293), .c(new_n290), .d(new_n292), .o1(\s[30] ));
  nano23aa1n06x5               g203(.a(new_n282), .b(new_n272), .c(new_n293), .d(new_n271), .out0(new_n299));
  inv000aa1d42x5               g204(.a(new_n299), .o1(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[30] ), .b(\a[31] ), .out0(new_n301));
  aoi012aa1n02x5               g206(.a(new_n296), .b(\a[30] ), .c(\b[29] ), .o1(new_n302));
  norp02aa1n02x5               g207(.a(new_n302), .b(new_n301), .o1(new_n303));
  aoai13aa1n02x5               g208(.a(new_n303), .b(new_n300), .c(new_n274), .d(new_n267), .o1(new_n304));
  aoai13aa1n03x5               g209(.a(new_n301), .b(new_n302), .c(new_n268), .d(new_n299), .o1(new_n305));
  nanp02aa1n03x5               g210(.a(new_n305), .b(new_n304), .o1(\s[31] ));
  inv000aa1d42x5               g211(.a(new_n105), .o1(new_n307));
  xnbna2aa1n03x5               g212(.a(new_n102), .b(new_n106), .c(new_n307), .out0(\s[3] ));
  norb02aa1n02x5               g213(.a(new_n104), .b(new_n103), .out0(new_n309));
  nanb03aa1n02x5               g214(.a(new_n102), .b(new_n106), .c(new_n307), .out0(new_n310));
  xnbna2aa1n03x5               g215(.a(new_n309), .b(new_n310), .c(new_n307), .out0(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g217(.a(new_n115), .b(new_n114), .out0(new_n313));
  aoai13aa1n02x5               g218(.a(new_n313), .b(new_n112), .c(new_n109), .d(new_n113), .o1(new_n314));
  aoi112aa1n02x5               g219(.a(new_n112), .b(new_n313), .c(new_n109), .d(new_n113), .o1(new_n315));
  nanb02aa1n02x5               g220(.a(new_n315), .b(new_n314), .out0(\s[6] ));
  oaib12aa1n02x5               g221(.a(new_n118), .b(new_n116), .c(new_n109), .out0(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanb02aa1n02x5               g223(.a(new_n111), .b(new_n317), .out0(new_n319));
  xobna2aa1n03x5               g224(.a(new_n110), .b(new_n319), .c(new_n120), .out0(\s[8] ));
  xnbna2aa1n03x5               g225(.a(new_n123), .b(new_n127), .c(new_n129), .out0(\s[9] ));
endmodule


