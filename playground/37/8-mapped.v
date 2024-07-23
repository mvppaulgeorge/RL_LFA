// Benchmark "adder" written by ABC on Thu Jul 18 06:57:37 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n185, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n308, new_n310, new_n312,
    new_n314, new_n316;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nand42aa1d28x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nor042aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nor042aa1n03x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nand42aa1n16x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nano23aa1n09x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .d(new_n99), .out0(new_n103));
  and002aa1n12x5               g008(.a(\b[5] ), .b(\a[6] ), .o(new_n104));
  nor002aa1d32x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nor002aa1d32x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  oab012aa1n09x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .out0(new_n107));
  oai022aa1n09x5               g012(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n108));
  aoi022aa1n12x5               g013(.a(new_n103), .b(new_n107), .c(new_n99), .d(new_n108), .o1(new_n109));
  nor042aa1n04x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  nor042aa1n06x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nanp02aa1n24x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  aoi012aa1n09x5               g017(.a(new_n110), .b(new_n111), .c(new_n112), .o1(new_n113));
  norb02aa1n09x5               g018(.a(new_n112), .b(new_n110), .out0(new_n114));
  nand42aa1n04x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  norb02aa1n06x5               g020(.a(new_n115), .b(new_n111), .out0(new_n116));
  and002aa1n12x5               g021(.a(\b[1] ), .b(\a[2] ), .o(new_n117));
  nand02aa1d24x5               g022(.a(\b[0] ), .b(\a[1] ), .o1(new_n118));
  nor042aa1n06x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  oab012aa1n12x5               g024(.a(new_n117), .b(new_n119), .c(new_n118), .out0(new_n120));
  nand23aa1n09x5               g025(.a(new_n120), .b(new_n114), .c(new_n116), .o1(new_n121));
  nand22aa1n02x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  nand02aa1d04x5               g027(.a(\b[4] ), .b(\a[5] ), .o1(new_n123));
  nano23aa1n06x5               g028(.a(new_n106), .b(new_n105), .c(new_n123), .d(new_n122), .out0(new_n124));
  nand22aa1n03x5               g029(.a(new_n124), .b(new_n103), .o1(new_n125));
  aoai13aa1n12x5               g030(.a(new_n109), .b(new_n125), .c(new_n121), .d(new_n113), .o1(new_n126));
  xorc02aa1n02x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoi012aa1n02x5               g032(.a(new_n98), .b(new_n126), .c(new_n127), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  inv000aa1d42x5               g034(.a(\a[9] ), .o1(new_n130));
  inv040aa1d32x5               g035(.a(\b[9] ), .o1(new_n131));
  xroi22aa1d04x5               g036(.a(new_n130), .b(\b[8] ), .c(new_n131), .d(\a[10] ), .out0(new_n132));
  oaoi03aa1n02x5               g037(.a(new_n97), .b(new_n131), .c(new_n98), .o1(new_n133));
  aob012aa1n03x5               g038(.a(new_n133), .b(new_n126), .c(new_n132), .out0(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n06x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand42aa1n08x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  tech160nm_fiaoi012aa1n05x5   g042(.a(new_n136), .b(new_n134), .c(new_n137), .o1(new_n138));
  xnrb03aa1n03x5               g043(.a(new_n138), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  oao003aa1n02x5               g044(.a(new_n97), .b(new_n131), .c(new_n98), .carry(new_n140));
  nor042aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanp02aa1n04x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nano23aa1n03x7               g047(.a(new_n136), .b(new_n141), .c(new_n142), .d(new_n137), .out0(new_n143));
  oai012aa1n02x5               g048(.a(new_n142), .b(new_n141), .c(new_n136), .o1(new_n144));
  aobi12aa1n06x5               g049(.a(new_n144), .b(new_n143), .c(new_n140), .out0(new_n145));
  nona23aa1n03x5               g050(.a(new_n142), .b(new_n137), .c(new_n136), .d(new_n141), .out0(new_n146));
  norb02aa1n02x5               g051(.a(new_n132), .b(new_n146), .out0(new_n147));
  nanp02aa1n03x5               g052(.a(new_n126), .b(new_n147), .o1(new_n148));
  nanp02aa1n03x5               g053(.a(new_n148), .b(new_n145), .o1(new_n149));
  xorb03aa1n02x5               g054(.a(new_n149), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n12x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nand22aa1n06x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  tech160nm_fiaoi012aa1n05x5   g057(.a(new_n151), .b(new_n149), .c(new_n152), .o1(new_n153));
  xnrb03aa1n03x5               g058(.a(new_n153), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1d32x5               g059(.a(\b[14] ), .b(\a[15] ), .o1(new_n155));
  nand02aa1d12x5               g060(.a(\b[14] ), .b(\a[15] ), .o1(new_n156));
  nanb02aa1d24x5               g061(.a(new_n155), .b(new_n156), .out0(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  tech160nm_fioai012aa1n03p5x5 g063(.a(new_n144), .b(new_n146), .c(new_n133), .o1(new_n159));
  nor042aa1n06x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nand22aa1n06x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nano23aa1n06x5               g066(.a(new_n151), .b(new_n160), .c(new_n161), .d(new_n152), .out0(new_n162));
  aoai13aa1n03x5               g067(.a(new_n162), .b(new_n159), .c(new_n126), .d(new_n147), .o1(new_n163));
  oaih12aa1n06x5               g068(.a(new_n161), .b(new_n160), .c(new_n151), .o1(new_n164));
  xnbna2aa1n03x5               g069(.a(new_n158), .b(new_n163), .c(new_n164), .out0(\s[15] ));
  nanp02aa1n03x5               g070(.a(new_n163), .b(new_n164), .o1(new_n166));
  nor042aa1n06x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nanp02aa1n04x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanb02aa1n06x5               g073(.a(new_n167), .b(new_n168), .out0(new_n169));
  aoai13aa1n03x5               g074(.a(new_n169), .b(new_n155), .c(new_n166), .d(new_n158), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(new_n166), .b(new_n158), .o1(new_n171));
  nona22aa1n02x4               g076(.a(new_n171), .b(new_n169), .c(new_n155), .out0(new_n172));
  nanp02aa1n02x5               g077(.a(new_n172), .b(new_n170), .o1(\s[16] ));
  oaih12aa1n02x5               g078(.a(new_n168), .b(new_n167), .c(new_n155), .o1(new_n174));
  oai013aa1n06x5               g079(.a(new_n174), .b(new_n164), .c(new_n157), .d(new_n169), .o1(new_n175));
  nona22aa1n09x5               g080(.a(new_n162), .b(new_n169), .c(new_n157), .out0(new_n176));
  aoib12aa1n12x5               g081(.a(new_n175), .b(new_n159), .c(new_n176), .out0(new_n177));
  nano22aa1n12x5               g082(.a(new_n176), .b(new_n132), .c(new_n143), .out0(new_n178));
  nanp02aa1n09x5               g083(.a(new_n126), .b(new_n178), .o1(new_n179));
  xorc02aa1n02x5               g084(.a(\a[17] ), .b(\b[16] ), .out0(new_n180));
  xnbna2aa1n03x5               g085(.a(new_n180), .b(new_n179), .c(new_n177), .out0(\s[17] ));
  inv040aa1d32x5               g086(.a(\a[18] ), .o1(new_n182));
  nanp02aa1n06x5               g087(.a(new_n179), .b(new_n177), .o1(new_n183));
  nor042aa1n03x5               g088(.a(\b[16] ), .b(\a[17] ), .o1(new_n184));
  tech160nm_fiaoi012aa1n05x5   g089(.a(new_n184), .b(new_n183), .c(new_n180), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[17] ), .c(new_n182), .out0(\s[18] ));
  oabi12aa1n09x5               g091(.a(new_n175), .b(new_n145), .c(new_n176), .out0(new_n187));
  inv000aa1d42x5               g092(.a(\a[17] ), .o1(new_n188));
  xroi22aa1d04x5               g093(.a(new_n188), .b(\b[16] ), .c(new_n182), .d(\b[17] ), .out0(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n187), .c(new_n126), .d(new_n178), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[17] ), .o1(new_n191));
  oaoi03aa1n09x5               g096(.a(new_n182), .b(new_n191), .c(new_n184), .o1(new_n192));
  nor002aa1n20x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nanp02aa1n04x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nanb02aa1n02x5               g099(.a(new_n193), .b(new_n194), .out0(new_n195));
  inv000aa1d42x5               g100(.a(new_n195), .o1(new_n196));
  xnbna2aa1n03x5               g101(.a(new_n196), .b(new_n190), .c(new_n192), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g103(.a(new_n190), .b(new_n192), .o1(new_n199));
  nor002aa1n10x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nand02aa1n04x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nanb02aa1n02x5               g106(.a(new_n200), .b(new_n201), .out0(new_n202));
  aoai13aa1n03x5               g107(.a(new_n202), .b(new_n193), .c(new_n199), .d(new_n196), .o1(new_n203));
  nanp02aa1n02x5               g108(.a(new_n199), .b(new_n196), .o1(new_n204));
  nona22aa1n02x4               g109(.a(new_n204), .b(new_n202), .c(new_n193), .out0(new_n205));
  nanp02aa1n02x5               g110(.a(new_n205), .b(new_n203), .o1(\s[20] ));
  nona23aa1n09x5               g111(.a(new_n201), .b(new_n194), .c(new_n193), .d(new_n200), .out0(new_n207));
  oa0012aa1n03x5               g112(.a(new_n201), .b(new_n200), .c(new_n193), .o(new_n208));
  inv030aa1n04x5               g113(.a(new_n208), .o1(new_n209));
  oai012aa1d24x5               g114(.a(new_n209), .b(new_n207), .c(new_n192), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  nano23aa1n09x5               g116(.a(new_n193), .b(new_n200), .c(new_n201), .d(new_n194), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(new_n189), .b(new_n212), .o1(new_n213));
  aoai13aa1n06x5               g118(.a(new_n211), .b(new_n213), .c(new_n179), .d(new_n177), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  xorc02aa1n02x5               g122(.a(\a[22] ), .b(\b[21] ), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  aoai13aa1n03x5               g124(.a(new_n219), .b(new_n216), .c(new_n214), .d(new_n217), .o1(new_n220));
  nand42aa1n02x5               g125(.a(new_n214), .b(new_n217), .o1(new_n221));
  nona22aa1n03x5               g126(.a(new_n221), .b(new_n219), .c(new_n216), .out0(new_n222));
  nanp02aa1n03x5               g127(.a(new_n222), .b(new_n220), .o1(\s[22] ));
  inv000aa1d42x5               g128(.a(\a[21] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\a[22] ), .o1(new_n225));
  xroi22aa1d06x4               g130(.a(new_n224), .b(\b[20] ), .c(new_n225), .d(\b[21] ), .out0(new_n226));
  inv000aa1d42x5               g131(.a(\b[21] ), .o1(new_n227));
  oao003aa1n02x5               g132(.a(new_n225), .b(new_n227), .c(new_n216), .carry(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n210), .c(new_n226), .o1(new_n229));
  nand23aa1n02x5               g134(.a(new_n226), .b(new_n189), .c(new_n212), .o1(new_n230));
  aoai13aa1n06x5               g135(.a(new_n229), .b(new_n230), .c(new_n179), .d(new_n177), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  tech160nm_fixorc02aa1n05x5   g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  xnrc02aa1n12x5               g139(.a(\b[23] ), .b(\a[24] ), .out0(new_n235));
  aoai13aa1n03x5               g140(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n236));
  nand42aa1n02x5               g141(.a(new_n231), .b(new_n234), .o1(new_n237));
  nona22aa1n02x4               g142(.a(new_n237), .b(new_n235), .c(new_n233), .out0(new_n238));
  nanp02aa1n03x5               g143(.a(new_n238), .b(new_n236), .o1(\s[24] ));
  norb02aa1n06x4               g144(.a(new_n234), .b(new_n235), .out0(new_n240));
  nano22aa1n02x4               g145(.a(new_n213), .b(new_n240), .c(new_n226), .out0(new_n241));
  aoai13aa1n03x5               g146(.a(new_n241), .b(new_n187), .c(new_n126), .d(new_n178), .o1(new_n242));
  oao003aa1n02x5               g147(.a(new_n182), .b(new_n191), .c(new_n184), .carry(new_n243));
  aoai13aa1n06x5               g148(.a(new_n226), .b(new_n208), .c(new_n212), .d(new_n243), .o1(new_n244));
  inv000aa1n02x5               g149(.a(new_n228), .o1(new_n245));
  inv000aa1n02x5               g150(.a(new_n240), .o1(new_n246));
  orn002aa1n03x5               g151(.a(\a[23] ), .b(\b[22] ), .o(new_n247));
  tech160nm_fioaoi03aa1n03p5x5 g152(.a(\a[24] ), .b(\b[23] ), .c(new_n247), .o1(new_n248));
  inv040aa1d30x5               g153(.a(new_n248), .o1(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n246), .c(new_n244), .d(new_n245), .o1(new_n250));
  nanb02aa1n03x5               g155(.a(new_n250), .b(new_n242), .out0(new_n251));
  xorb03aa1n02x5               g156(.a(new_n251), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  xorc02aa1n03x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  nor042aa1n03x5               g159(.a(\b[25] ), .b(\a[26] ), .o1(new_n255));
  nand02aa1d24x5               g160(.a(\b[25] ), .b(\a[26] ), .o1(new_n256));
  norb02aa1n03x4               g161(.a(new_n256), .b(new_n255), .out0(new_n257));
  inv030aa1n04x5               g162(.a(new_n257), .o1(new_n258));
  aoai13aa1n02x5               g163(.a(new_n258), .b(new_n253), .c(new_n251), .d(new_n254), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n254), .b(new_n250), .c(new_n183), .d(new_n241), .o1(new_n260));
  nona22aa1n03x5               g165(.a(new_n260), .b(new_n258), .c(new_n253), .out0(new_n261));
  nanp02aa1n03x5               g166(.a(new_n259), .b(new_n261), .o1(\s[26] ));
  norb02aa1d21x5               g167(.a(new_n254), .b(new_n258), .out0(new_n263));
  nanp02aa1n02x5               g168(.a(new_n250), .b(new_n263), .o1(new_n264));
  oai012aa1n02x5               g169(.a(new_n256), .b(new_n255), .c(new_n253), .o1(new_n265));
  nano22aa1n03x7               g170(.a(new_n230), .b(new_n240), .c(new_n263), .out0(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n187), .c(new_n126), .d(new_n178), .o1(new_n267));
  nand23aa1n04x5               g172(.a(new_n264), .b(new_n267), .c(new_n265), .o1(new_n268));
  xorb03aa1n02x5               g173(.a(new_n268), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  xorc02aa1n02x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[27] ), .b(\a[28] ), .out0(new_n272));
  aoai13aa1n03x5               g177(.a(new_n272), .b(new_n270), .c(new_n268), .d(new_n271), .o1(new_n273));
  aoai13aa1n06x5               g178(.a(new_n240), .b(new_n228), .c(new_n210), .d(new_n226), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n263), .o1(new_n275));
  aoai13aa1n04x5               g180(.a(new_n265), .b(new_n275), .c(new_n274), .d(new_n249), .o1(new_n276));
  aobi12aa1n09x5               g181(.a(new_n266), .b(new_n179), .c(new_n177), .out0(new_n277));
  oaih12aa1n02x5               g182(.a(new_n271), .b(new_n276), .c(new_n277), .o1(new_n278));
  nona22aa1n03x5               g183(.a(new_n278), .b(new_n272), .c(new_n270), .out0(new_n279));
  nanp02aa1n03x5               g184(.a(new_n273), .b(new_n279), .o1(\s[28] ));
  norb02aa1n02x5               g185(.a(new_n271), .b(new_n272), .out0(new_n281));
  oaih12aa1n02x5               g186(.a(new_n281), .b(new_n276), .c(new_n277), .o1(new_n282));
  inv000aa1n03x5               g187(.a(new_n270), .o1(new_n283));
  oaoi03aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .o1(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[28] ), .b(\a[29] ), .out0(new_n285));
  nona22aa1n03x5               g190(.a(new_n282), .b(new_n284), .c(new_n285), .out0(new_n286));
  aoai13aa1n03x5               g191(.a(new_n285), .b(new_n284), .c(new_n268), .d(new_n281), .o1(new_n287));
  nanp02aa1n03x5               g192(.a(new_n287), .b(new_n286), .o1(\s[29] ));
  xorb03aa1n02x5               g193(.a(new_n118), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g194(.a(new_n271), .b(new_n285), .c(new_n272), .out0(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n291));
  oaoi03aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .o1(new_n292));
  tech160nm_fixorc02aa1n03p5x5 g197(.a(\a[30] ), .b(\b[29] ), .out0(new_n293));
  inv000aa1d42x5               g198(.a(new_n293), .o1(new_n294));
  aoai13aa1n02x7               g199(.a(new_n294), .b(new_n292), .c(new_n268), .d(new_n290), .o1(new_n295));
  oaih12aa1n02x5               g200(.a(new_n290), .b(new_n276), .c(new_n277), .o1(new_n296));
  nona22aa1n03x5               g201(.a(new_n296), .b(new_n292), .c(new_n294), .out0(new_n297));
  nanp02aa1n03x5               g202(.a(new_n295), .b(new_n297), .o1(\s[30] ));
  nanp02aa1n02x5               g203(.a(new_n292), .b(new_n293), .o1(new_n299));
  oai012aa1n02x5               g204(.a(new_n299), .b(\b[29] ), .c(\a[30] ), .o1(new_n300));
  nano23aa1n02x4               g205(.a(new_n285), .b(new_n272), .c(new_n293), .d(new_n271), .out0(new_n301));
  oaih12aa1n02x5               g206(.a(new_n301), .b(new_n276), .c(new_n277), .o1(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  nona22aa1n03x5               g208(.a(new_n302), .b(new_n303), .c(new_n300), .out0(new_n304));
  aoai13aa1n02x7               g209(.a(new_n303), .b(new_n300), .c(new_n268), .d(new_n301), .o1(new_n305));
  nanp02aa1n03x5               g210(.a(new_n305), .b(new_n304), .o1(\s[31] ));
  xorb03aa1n02x5               g211(.a(new_n120), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oai012aa1n02x5               g212(.a(new_n115), .b(new_n120), .c(new_n111), .o1(new_n308));
  xnrc02aa1n02x5               g213(.a(new_n308), .b(new_n114), .out0(\s[4] ));
  nanp02aa1n02x5               g214(.a(new_n121), .b(new_n113), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g216(.a(new_n106), .b(new_n310), .c(new_n123), .o1(new_n312));
  xnrb03aa1n02x5               g217(.a(new_n312), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g218(.a(new_n107), .b(new_n310), .c(new_n124), .o(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g220(.a(new_n101), .b(new_n314), .c(new_n102), .o1(new_n316));
  xnrb03aa1n02x5               g221(.a(new_n316), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g222(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule

