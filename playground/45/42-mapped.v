// Benchmark "adder" written by ABC on Thu Jul 18 11:26:19 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n152, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n160, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n177, new_n178, new_n179, new_n180, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n210, new_n211, new_n212, new_n213, new_n214,
    new_n215, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n230,
    new_n231, new_n232, new_n233, new_n234, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n245, new_n246,
    new_n247, new_n248, new_n249, new_n250, new_n251, new_n252, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n305, new_n308, new_n309, new_n311, new_n312, new_n313;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  orn002aa1n02x5               g002(.a(\a[9] ), .b(\b[8] ), .o(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  tech160nm_finand02aa1n03p5x5 g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  tech160nm_fioaoi03aa1n03p5x5 g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nor002aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n06x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n06x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  oai012aa1n02x5               g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  oai012aa1n12x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nor022aa1n04x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand22aa1n03x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  norp02aa1n12x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nand42aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nona23aa1n06x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  nand42aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  tech160nm_finor002aa1n05x5   g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nanb02aa1n09x5               g021(.a(new_n116), .b(new_n115), .out0(new_n117));
  tech160nm_fixorc02aa1n02p5x5 g022(.a(\a[8] ), .b(\b[7] ), .out0(new_n118));
  norb03aa1n09x5               g023(.a(new_n118), .b(new_n114), .c(new_n117), .out0(new_n119));
  oaoi13aa1n06x5               g024(.a(new_n110), .b(new_n115), .c(new_n112), .d(new_n116), .o1(new_n120));
  aob012aa1n02x5               g025(.a(new_n111), .b(\b[7] ), .c(\a[8] ), .out0(new_n121));
  oai022aa1n12x5               g026(.a(new_n120), .b(new_n121), .c(\b[7] ), .d(\a[8] ), .o1(new_n122));
  xorc02aa1n12x5               g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n119), .d(new_n109), .o1(new_n124));
  xnbna2aa1n03x5               g029(.a(new_n97), .b(new_n124), .c(new_n98), .out0(\s[10] ));
  nanp02aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand42aa1n04x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  nor002aa1n03x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  nanb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  oai022aa1n09x5               g034(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n130));
  nanb02aa1n03x5               g035(.a(new_n130), .b(new_n124), .out0(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n129), .b(new_n131), .c(new_n126), .out0(\s[11] ));
  inv000aa1d42x5               g037(.a(\a[12] ), .o1(new_n133));
  aoi013aa1n02x4               g038(.a(new_n128), .b(new_n131), .c(new_n126), .d(new_n127), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[11] ), .c(new_n133), .out0(\s[12] ));
  nor002aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nano23aa1n06x5               g042(.a(new_n136), .b(new_n128), .c(new_n137), .d(new_n127), .out0(new_n138));
  nand23aa1d12x5               g043(.a(new_n138), .b(new_n97), .c(new_n123), .o1(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  aoai13aa1n06x5               g045(.a(new_n140), .b(new_n122), .c(new_n119), .d(new_n109), .o1(new_n141));
  aoi022aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n142));
  aoai13aa1n04x5               g047(.a(new_n142), .b(new_n128), .c(new_n130), .d(new_n126), .o1(new_n143));
  oaib12aa1n09x5               g048(.a(new_n143), .b(\b[11] ), .c(new_n133), .out0(new_n144));
  inv000aa1d42x5               g049(.a(new_n144), .o1(new_n145));
  xnrc02aa1n12x5               g050(.a(\b[12] ), .b(\a[13] ), .out0(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  xnbna2aa1n03x5               g052(.a(new_n147), .b(new_n141), .c(new_n145), .out0(\s[13] ));
  inv000aa1d42x5               g053(.a(\a[14] ), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n141), .b(new_n145), .o1(new_n150));
  nor042aa1n03x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  aoi012aa1n02x5               g056(.a(new_n151), .b(new_n150), .c(new_n147), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[13] ), .c(new_n149), .out0(\s[14] ));
  xnrc02aa1n02x5               g058(.a(\b[13] ), .b(\a[14] ), .out0(new_n154));
  nanb02aa1n02x5               g059(.a(new_n154), .b(new_n147), .out0(new_n155));
  inv000aa1d42x5               g060(.a(\b[13] ), .o1(new_n156));
  oaoi03aa1n03x5               g061(.a(new_n149), .b(new_n156), .c(new_n151), .o1(new_n157));
  aoai13aa1n04x5               g062(.a(new_n157), .b(new_n155), .c(new_n141), .d(new_n145), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n03x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  nor042aa1n02x5               g066(.a(\b[15] ), .b(\a[16] ), .o1(new_n162));
  nand42aa1n04x5               g067(.a(\b[15] ), .b(\a[16] ), .o1(new_n163));
  nanb02aa1n02x5               g068(.a(new_n162), .b(new_n163), .out0(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n160), .c(new_n158), .d(new_n161), .o1(new_n165));
  aoi112aa1n02x5               g070(.a(new_n160), .b(new_n164), .c(new_n158), .d(new_n161), .o1(new_n166));
  nanb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(\s[16] ));
  nano23aa1n03x5               g072(.a(new_n160), .b(new_n162), .c(new_n163), .d(new_n161), .out0(new_n168));
  nona22aa1n03x5               g073(.a(new_n168), .b(new_n154), .c(new_n146), .out0(new_n169));
  nor042aa1n06x5               g074(.a(new_n169), .b(new_n139), .o1(new_n170));
  aoai13aa1n12x5               g075(.a(new_n170), .b(new_n122), .c(new_n109), .d(new_n119), .o1(new_n171));
  norp02aa1n02x5               g076(.a(new_n162), .b(new_n160), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n157), .c(\a[15] ), .d(\b[14] ), .o1(new_n173));
  aboi22aa1n12x5               g078(.a(new_n169), .b(new_n144), .c(new_n173), .d(new_n163), .out0(new_n174));
  nanp02aa1n06x5               g079(.a(new_n171), .b(new_n174), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g081(.a(\a[18] ), .o1(new_n177));
  inv000aa1d42x5               g082(.a(\a[17] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\b[16] ), .o1(new_n179));
  oaoi03aa1n03x5               g084(.a(new_n178), .b(new_n179), .c(new_n175), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[17] ), .c(new_n177), .out0(\s[18] ));
  xroi22aa1d04x5               g086(.a(new_n178), .b(\b[16] ), .c(new_n177), .d(\b[17] ), .out0(new_n182));
  nanp02aa1n02x5               g087(.a(new_n179), .b(new_n178), .o1(new_n183));
  oaoi03aa1n02x5               g088(.a(\a[18] ), .b(\b[17] ), .c(new_n183), .o1(new_n184));
  nor022aa1n16x5               g089(.a(\b[18] ), .b(\a[19] ), .o1(new_n185));
  nand42aa1n04x5               g090(.a(\b[18] ), .b(\a[19] ), .o1(new_n186));
  nanb02aa1n02x5               g091(.a(new_n185), .b(new_n186), .out0(new_n187));
  inv000aa1d42x5               g092(.a(new_n187), .o1(new_n188));
  aoai13aa1n06x5               g093(.a(new_n188), .b(new_n184), .c(new_n175), .d(new_n182), .o1(new_n189));
  aoi112aa1n02x5               g094(.a(new_n188), .b(new_n184), .c(new_n175), .d(new_n182), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n189), .b(new_n190), .out0(\s[19] ));
  xnrc02aa1n02x5               g096(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g097(.a(\b[19] ), .b(\a[20] ), .o1(new_n193));
  nand22aa1n04x5               g098(.a(\b[19] ), .b(\a[20] ), .o1(new_n194));
  nanb02aa1n02x5               g099(.a(new_n193), .b(new_n194), .out0(new_n195));
  tech160nm_fioai012aa1n03p5x5 g100(.a(new_n189), .b(\b[18] ), .c(\a[19] ), .o1(new_n196));
  nanp02aa1n03x5               g101(.a(new_n196), .b(new_n195), .o1(new_n197));
  nona22aa1n02x5               g102(.a(new_n189), .b(new_n195), .c(new_n185), .out0(new_n198));
  nanp02aa1n03x5               g103(.a(new_n197), .b(new_n198), .o1(\s[20] ));
  nano23aa1n09x5               g104(.a(new_n185), .b(new_n193), .c(new_n194), .d(new_n186), .out0(new_n200));
  nanp02aa1n02x5               g105(.a(new_n182), .b(new_n200), .o1(new_n201));
  oai022aa1n02x5               g106(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n202));
  oaib12aa1n02x5               g107(.a(new_n202), .b(new_n177), .c(\b[17] ), .out0(new_n203));
  nona23aa1n09x5               g108(.a(new_n194), .b(new_n186), .c(new_n185), .d(new_n193), .out0(new_n204));
  aoi012aa1n09x5               g109(.a(new_n193), .b(new_n185), .c(new_n194), .o1(new_n205));
  oai012aa1n12x5               g110(.a(new_n205), .b(new_n204), .c(new_n203), .o1(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  aoai13aa1n04x5               g112(.a(new_n207), .b(new_n201), .c(new_n171), .d(new_n174), .o1(new_n208));
  xorb03aa1n02x5               g113(.a(new_n208), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g114(.a(\b[20] ), .b(\a[21] ), .o1(new_n210));
  xorc02aa1n02x5               g115(.a(\a[21] ), .b(\b[20] ), .out0(new_n211));
  xorc02aa1n02x5               g116(.a(\a[22] ), .b(\b[21] ), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  aoai13aa1n02x5               g118(.a(new_n213), .b(new_n210), .c(new_n208), .d(new_n211), .o1(new_n214));
  aoi112aa1n03x4               g119(.a(new_n210), .b(new_n213), .c(new_n208), .d(new_n211), .o1(new_n215));
  nanb02aa1n02x5               g120(.a(new_n215), .b(new_n214), .out0(\s[22] ));
  inv000aa1d42x5               g121(.a(\a[21] ), .o1(new_n217));
  inv000aa1d42x5               g122(.a(\a[22] ), .o1(new_n218));
  xroi22aa1d04x5               g123(.a(new_n217), .b(\b[20] ), .c(new_n218), .d(\b[21] ), .out0(new_n219));
  nanp03aa1n02x5               g124(.a(new_n219), .b(new_n182), .c(new_n200), .o1(new_n220));
  inv000aa1n02x5               g125(.a(new_n205), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n219), .b(new_n221), .c(new_n200), .d(new_n184), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\b[21] ), .o1(new_n223));
  oao003aa1n02x5               g128(.a(new_n218), .b(new_n223), .c(new_n210), .carry(new_n224));
  inv000aa1n02x5               g129(.a(new_n224), .o1(new_n225));
  nanp02aa1n02x5               g130(.a(new_n222), .b(new_n225), .o1(new_n226));
  inv000aa1n02x5               g131(.a(new_n226), .o1(new_n227));
  aoai13aa1n04x5               g132(.a(new_n227), .b(new_n220), .c(new_n171), .d(new_n174), .o1(new_n228));
  xorb03aa1n02x5               g133(.a(new_n228), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g134(.a(\b[22] ), .b(\a[23] ), .o1(new_n230));
  xorc02aa1n02x5               g135(.a(\a[23] ), .b(\b[22] ), .out0(new_n231));
  xnrc02aa1n02x5               g136(.a(\b[23] ), .b(\a[24] ), .out0(new_n232));
  aoai13aa1n02x5               g137(.a(new_n232), .b(new_n230), .c(new_n228), .d(new_n231), .o1(new_n233));
  aoi112aa1n03x4               g138(.a(new_n230), .b(new_n232), .c(new_n228), .d(new_n231), .o1(new_n234));
  nanb02aa1n02x5               g139(.a(new_n234), .b(new_n233), .out0(\s[24] ));
  norb02aa1n02x5               g140(.a(new_n231), .b(new_n232), .out0(new_n236));
  nanb03aa1n02x5               g141(.a(new_n201), .b(new_n236), .c(new_n219), .out0(new_n237));
  inv000aa1n02x5               g142(.a(new_n236), .o1(new_n238));
  oai022aa1n02x5               g143(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n239));
  aob012aa1n02x5               g144(.a(new_n239), .b(\b[23] ), .c(\a[24] ), .out0(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n238), .c(new_n222), .d(new_n225), .o1(new_n241));
  inv040aa1n03x5               g146(.a(new_n241), .o1(new_n242));
  aoai13aa1n04x5               g147(.a(new_n242), .b(new_n237), .c(new_n171), .d(new_n174), .o1(new_n243));
  xorb03aa1n02x5               g148(.a(new_n243), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g149(.a(\b[24] ), .b(\a[25] ), .o1(new_n245));
  xorc02aa1n12x5               g150(.a(\a[25] ), .b(\b[24] ), .out0(new_n246));
  nor042aa1n03x5               g151(.a(\b[25] ), .b(\a[26] ), .o1(new_n247));
  nand42aa1n03x5               g152(.a(\b[25] ), .b(\a[26] ), .o1(new_n248));
  norb02aa1n03x5               g153(.a(new_n248), .b(new_n247), .out0(new_n249));
  inv040aa1n03x5               g154(.a(new_n249), .o1(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n245), .c(new_n243), .d(new_n246), .o1(new_n251));
  aoi112aa1n03x4               g156(.a(new_n245), .b(new_n250), .c(new_n243), .d(new_n246), .o1(new_n252));
  nanb02aa1n03x5               g157(.a(new_n252), .b(new_n251), .out0(\s[26] ));
  nanp02aa1n02x5               g158(.a(new_n119), .b(new_n109), .o1(new_n254));
  oa0022aa1n02x5               g159(.a(new_n120), .b(new_n121), .c(\a[8] ), .d(\b[7] ), .o(new_n255));
  nanp02aa1n02x5               g160(.a(new_n254), .b(new_n255), .o1(new_n256));
  nanp02aa1n02x5               g161(.a(new_n173), .b(new_n163), .o1(new_n257));
  oaib12aa1n02x5               g162(.a(new_n257), .b(new_n169), .c(new_n144), .out0(new_n258));
  norb02aa1n06x5               g163(.a(new_n246), .b(new_n250), .out0(new_n259));
  nano22aa1n03x7               g164(.a(new_n220), .b(new_n236), .c(new_n259), .out0(new_n260));
  aoai13aa1n04x5               g165(.a(new_n260), .b(new_n258), .c(new_n256), .d(new_n170), .o1(new_n261));
  oai012aa1n02x5               g166(.a(new_n248), .b(new_n247), .c(new_n245), .o1(new_n262));
  aobi12aa1n06x5               g167(.a(new_n262), .b(new_n241), .c(new_n259), .out0(new_n263));
  xorc02aa1n02x5               g168(.a(\a[27] ), .b(\b[26] ), .out0(new_n264));
  xnbna2aa1n03x5               g169(.a(new_n264), .b(new_n263), .c(new_n261), .out0(\s[27] ));
  nand42aa1n03x5               g170(.a(new_n263), .b(new_n261), .o1(new_n266));
  norp02aa1n02x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  norp02aa1n02x5               g172(.a(\b[27] ), .b(\a[28] ), .o1(new_n268));
  nanp02aa1n02x5               g173(.a(\b[27] ), .b(\a[28] ), .o1(new_n269));
  norb02aa1n09x5               g174(.a(new_n269), .b(new_n268), .out0(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  aoai13aa1n03x5               g176(.a(new_n271), .b(new_n267), .c(new_n266), .d(new_n264), .o1(new_n272));
  aobi12aa1n06x5               g177(.a(new_n260), .b(new_n171), .c(new_n174), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n236), .b(new_n224), .c(new_n206), .d(new_n219), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n259), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n262), .b(new_n275), .c(new_n274), .d(new_n240), .o1(new_n276));
  oaih12aa1n02x5               g181(.a(new_n264), .b(new_n276), .c(new_n273), .o1(new_n277));
  nona22aa1n03x5               g182(.a(new_n277), .b(new_n271), .c(new_n267), .out0(new_n278));
  nanp02aa1n03x5               g183(.a(new_n272), .b(new_n278), .o1(\s[28] ));
  norb02aa1n02x5               g184(.a(new_n264), .b(new_n271), .out0(new_n280));
  oaih12aa1n02x5               g185(.a(new_n280), .b(new_n276), .c(new_n273), .o1(new_n281));
  xorc02aa1n02x5               g186(.a(\a[29] ), .b(\b[28] ), .out0(new_n282));
  aoi012aa1n02x5               g187(.a(new_n268), .b(new_n267), .c(new_n269), .o1(new_n283));
  norb02aa1n02x5               g188(.a(new_n283), .b(new_n282), .out0(new_n284));
  nand42aa1n02x5               g189(.a(new_n281), .b(new_n283), .o1(new_n285));
  aoi022aa1n02x7               g190(.a(new_n285), .b(new_n282), .c(new_n281), .d(new_n284), .o1(\s[29] ));
  xorb03aa1n02x5               g191(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g192(.a(new_n264), .b(new_n282), .c(new_n270), .o(new_n288));
  oaih12aa1n02x5               g193(.a(new_n288), .b(new_n276), .c(new_n273), .o1(new_n289));
  xorc02aa1n02x5               g194(.a(\a[30] ), .b(\b[29] ), .out0(new_n290));
  oao003aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .c(new_n283), .carry(new_n291));
  norb02aa1n02x5               g196(.a(new_n291), .b(new_n290), .out0(new_n292));
  nanp02aa1n03x5               g197(.a(new_n289), .b(new_n291), .o1(new_n293));
  aoi022aa1n02x7               g198(.a(new_n293), .b(new_n290), .c(new_n289), .d(new_n292), .o1(\s[30] ));
  and003aa1n02x5               g199(.a(new_n280), .b(new_n290), .c(new_n282), .o(new_n295));
  oaih12aa1n02x5               g200(.a(new_n295), .b(new_n276), .c(new_n273), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[30] ), .b(\b[29] ), .c(new_n291), .carry(new_n297));
  nanb02aa1n02x5               g202(.a(\b[30] ), .b(\a[31] ), .out0(new_n298));
  nanb02aa1n02x5               g203(.a(\a[31] ), .b(\b[30] ), .out0(new_n299));
  aoi022aa1n03x5               g204(.a(new_n296), .b(new_n297), .c(new_n299), .d(new_n298), .o1(new_n300));
  aobi12aa1n06x5               g205(.a(new_n295), .b(new_n263), .c(new_n261), .out0(new_n301));
  nano32aa1n02x4               g206(.a(new_n301), .b(new_n299), .c(new_n297), .d(new_n298), .out0(new_n302));
  norp02aa1n02x5               g207(.a(new_n300), .b(new_n302), .o1(\s[31] ));
  xnrb03aa1n02x5               g208(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g209(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n305));
  xorb03aa1n02x5               g210(.a(new_n305), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g211(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoai13aa1n02x5               g212(.a(new_n117), .b(new_n112), .c(new_n109), .d(new_n113), .o1(new_n308));
  aoi112aa1n03x5               g213(.a(new_n112), .b(new_n117), .c(new_n109), .d(new_n113), .o1(new_n309));
  nanb02aa1n02x5               g214(.a(new_n309), .b(new_n308), .out0(\s[6] ));
  norb02aa1n02x5               g215(.a(new_n111), .b(new_n110), .out0(new_n311));
  aoai13aa1n06x5               g216(.a(new_n311), .b(new_n309), .c(\b[5] ), .d(\a[6] ), .o1(new_n312));
  nona22aa1n02x4               g217(.a(new_n115), .b(new_n309), .c(new_n311), .out0(new_n313));
  nanp02aa1n02x5               g218(.a(new_n313), .b(new_n312), .o1(\s[7] ));
  xobna2aa1n03x5               g219(.a(new_n118), .b(new_n312), .c(new_n111), .out0(\s[8] ));
  xnbna2aa1n03x5               g220(.a(new_n123), .b(new_n254), .c(new_n255), .out0(\s[9] ));
endmodule


