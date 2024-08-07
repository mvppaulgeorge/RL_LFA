// Benchmark "adder" written by ABC on Wed Jul 17 14:52:17 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n318, new_n319, new_n320, new_n321, new_n323, new_n324,
    new_n325, new_n327, new_n328, new_n330, new_n331, new_n334, new_n336;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nor002aa1d32x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  inv030aa1d32x5               g006(.a(\a[2] ), .o1(new_n102));
  inv030aa1d32x5               g007(.a(\b[1] ), .o1(new_n103));
  nand22aa1n12x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  oaoi03aa1n12x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  tech160nm_finor002aa1n05x5   g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  tech160nm_finand02aa1n03p5x5 g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  tech160nm_finor002aa1n05x5   g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nand42aa1n03x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nona23aa1n09x5               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  inv000aa1d42x5               g015(.a(\a[3] ), .o1(new_n111));
  inv000aa1d42x5               g016(.a(\b[2] ), .o1(new_n112));
  aoai13aa1n04x5               g017(.a(new_n107), .b(new_n106), .c(new_n111), .d(new_n112), .o1(new_n113));
  oai012aa1d24x5               g018(.a(new_n113), .b(new_n110), .c(new_n105), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nand02aa1n06x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nor022aa1n12x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nand42aa1n03x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nona23aa1n03x5               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  xnrc02aa1n02x5               g024(.a(\b[7] ), .b(\a[8] ), .out0(new_n120));
  nor042aa1n06x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  nand42aa1n03x5               g026(.a(\b[6] ), .b(\a[7] ), .o1(new_n122));
  nanb02aa1n03x5               g027(.a(new_n121), .b(new_n122), .out0(new_n123));
  nor043aa1n06x5               g028(.a(new_n119), .b(new_n120), .c(new_n123), .o1(new_n124));
  tech160nm_fixorc02aa1n04x5   g029(.a(\a[8] ), .b(\b[7] ), .out0(new_n125));
  inv000aa1d42x5               g030(.a(\b[5] ), .o1(new_n126));
  nanb02aa1n12x5               g031(.a(\a[6] ), .b(new_n126), .out0(new_n127));
  oai112aa1n04x5               g032(.a(new_n127), .b(new_n116), .c(\b[4] ), .d(\a[5] ), .o1(new_n128));
  nano22aa1n06x5               g033(.a(new_n121), .b(new_n116), .c(new_n122), .out0(new_n129));
  nand03aa1n02x5               g034(.a(new_n129), .b(new_n125), .c(new_n128), .o1(new_n130));
  inv000aa1n02x5               g035(.a(new_n121), .o1(new_n131));
  oao003aa1n02x5               g036(.a(\a[8] ), .b(\b[7] ), .c(new_n131), .carry(new_n132));
  nand22aa1n09x5               g037(.a(new_n130), .b(new_n132), .o1(new_n133));
  nand42aa1n08x5               g038(.a(\b[8] ), .b(\a[9] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n100), .out0(new_n135));
  aoai13aa1n02x5               g040(.a(new_n135), .b(new_n133), .c(new_n114), .d(new_n124), .o1(new_n136));
  xobna2aa1n03x5               g041(.a(new_n99), .b(new_n136), .c(new_n101), .out0(\s[10] ));
  nano23aa1d15x5               g042(.a(new_n97), .b(new_n100), .c(new_n134), .d(new_n98), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n138), .b(new_n133), .c(new_n114), .d(new_n124), .o1(new_n139));
  aoi012aa1d18x5               g044(.a(new_n97), .b(new_n100), .c(new_n98), .o1(new_n140));
  nor002aa1d32x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nand02aa1d28x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n139), .c(new_n140), .out0(\s[11] ));
  inv000aa1d42x5               g049(.a(new_n141), .o1(new_n145));
  aob012aa1n02x5               g050(.a(new_n143), .b(new_n139), .c(new_n140), .out0(new_n146));
  norp02aa1n24x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nand02aa1d28x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  nona23aa1n02x4               g054(.a(new_n146), .b(new_n148), .c(new_n147), .d(new_n141), .out0(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n149), .c(new_n145), .d(new_n146), .o1(\s[12] ));
  nano23aa1n06x5               g056(.a(new_n141), .b(new_n147), .c(new_n148), .d(new_n142), .out0(new_n152));
  nand22aa1n09x5               g057(.a(new_n152), .b(new_n138), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n133), .c(new_n114), .d(new_n124), .o1(new_n155));
  nona23aa1d18x5               g060(.a(new_n148), .b(new_n142), .c(new_n141), .d(new_n147), .out0(new_n156));
  oai012aa1n04x7               g061(.a(new_n148), .b(new_n147), .c(new_n141), .o1(new_n157));
  oai012aa1d24x5               g062(.a(new_n157), .b(new_n156), .c(new_n140), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  nor042aa1n04x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nand02aa1n06x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  xnbna2aa1n03x5               g067(.a(new_n162), .b(new_n155), .c(new_n159), .out0(\s[13] ));
  nanp02aa1n06x5               g068(.a(new_n155), .b(new_n159), .o1(new_n164));
  nor002aa1n04x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nand42aa1n06x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nanb02aa1n02x5               g071(.a(new_n165), .b(new_n166), .out0(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n160), .c(new_n164), .d(new_n162), .o1(new_n168));
  nona22aa1n02x4               g073(.a(new_n166), .b(new_n165), .c(new_n160), .out0(new_n169));
  aoai13aa1n02x5               g074(.a(new_n168), .b(new_n169), .c(new_n162), .d(new_n164), .o1(\s[14] ));
  nano23aa1d12x5               g075(.a(new_n160), .b(new_n165), .c(new_n166), .d(new_n161), .out0(new_n171));
  oa0012aa1n06x5               g076(.a(new_n166), .b(new_n165), .c(new_n160), .o(new_n172));
  nor002aa1n12x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nand42aa1n03x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  aoai13aa1n06x5               g080(.a(new_n175), .b(new_n172), .c(new_n164), .d(new_n171), .o1(new_n176));
  aoi112aa1n02x5               g081(.a(new_n175), .b(new_n172), .c(new_n164), .d(new_n171), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n176), .b(new_n177), .out0(\s[15] ));
  inv000aa1d42x5               g083(.a(new_n173), .o1(new_n179));
  tech160nm_finor002aa1n05x5   g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  tech160nm_finand02aa1n05x5   g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  norb02aa1n02x5               g086(.a(new_n181), .b(new_n180), .out0(new_n182));
  nona23aa1n03x5               g087(.a(new_n176), .b(new_n181), .c(new_n180), .d(new_n173), .out0(new_n183));
  aoai13aa1n03x5               g088(.a(new_n183), .b(new_n182), .c(new_n176), .d(new_n179), .o1(\s[16] ));
  nano23aa1d12x5               g089(.a(new_n173), .b(new_n180), .c(new_n181), .d(new_n174), .out0(new_n185));
  nano22aa1n06x5               g090(.a(new_n153), .b(new_n171), .c(new_n185), .out0(new_n186));
  aoai13aa1n12x5               g091(.a(new_n186), .b(new_n133), .c(new_n114), .d(new_n124), .o1(new_n187));
  aoai13aa1n12x5               g092(.a(new_n185), .b(new_n172), .c(new_n158), .d(new_n171), .o1(new_n188));
  oai012aa1n12x5               g093(.a(new_n181), .b(new_n180), .c(new_n173), .o1(new_n189));
  nand23aa1d12x5               g094(.a(new_n187), .b(new_n188), .c(new_n189), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  xorc02aa1n02x5               g096(.a(\a[17] ), .b(\b[16] ), .out0(new_n192));
  nor042aa1n02x5               g097(.a(\b[16] ), .b(\a[17] ), .o1(new_n193));
  nor002aa1n02x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  nand22aa1n03x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  nanb02aa1n06x5               g100(.a(new_n194), .b(new_n195), .out0(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n193), .c(new_n190), .d(new_n192), .o1(new_n197));
  nona22aa1n02x4               g102(.a(new_n195), .b(new_n194), .c(new_n193), .out0(new_n198));
  aoai13aa1n02x5               g103(.a(new_n197), .b(new_n198), .c(new_n192), .d(new_n190), .o1(\s[18] ));
  norb02aa1n02x5               g104(.a(new_n192), .b(new_n196), .out0(new_n200));
  tech160nm_fiaoi012aa1n05x5   g105(.a(new_n194), .b(new_n193), .c(new_n195), .o1(new_n201));
  inv020aa1n02x5               g106(.a(new_n201), .o1(new_n202));
  nor042aa1n09x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nand42aa1n02x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n202), .c(new_n190), .d(new_n200), .o1(new_n206));
  aoi112aa1n02x5               g111(.a(new_n205), .b(new_n202), .c(new_n190), .d(new_n200), .o1(new_n207));
  norb02aa1n03x4               g112(.a(new_n206), .b(new_n207), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1n03x5               g114(.a(new_n203), .o1(new_n210));
  nor002aa1n03x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nand42aa1n02x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  norb02aa1n02x5               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  norb03aa1n02x5               g118(.a(new_n212), .b(new_n203), .c(new_n211), .out0(new_n214));
  nanp02aa1n06x5               g119(.a(new_n206), .b(new_n214), .o1(new_n215));
  aoai13aa1n03x5               g120(.a(new_n215), .b(new_n213), .c(new_n210), .d(new_n206), .o1(\s[20] ));
  nona23aa1n09x5               g121(.a(new_n212), .b(new_n204), .c(new_n203), .d(new_n211), .out0(new_n217));
  norb03aa1n09x5               g122(.a(new_n192), .b(new_n217), .c(new_n196), .out0(new_n218));
  oaoi03aa1n03x5               g123(.a(\a[20] ), .b(\b[19] ), .c(new_n210), .o1(new_n219));
  inv000aa1n02x5               g124(.a(new_n219), .o1(new_n220));
  oai012aa1n06x5               g125(.a(new_n220), .b(new_n217), .c(new_n201), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  aoai13aa1n06x5               g127(.a(new_n222), .b(new_n221), .c(new_n190), .d(new_n218), .o1(new_n223));
  aoi112aa1n02x5               g128(.a(new_n222), .b(new_n221), .c(new_n190), .d(new_n218), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n223), .b(new_n224), .out0(\s[21] ));
  nor042aa1n06x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  xorc02aa1n02x5               g132(.a(\a[22] ), .b(\b[21] ), .out0(new_n228));
  and002aa1n02x5               g133(.a(\b[21] ), .b(\a[22] ), .o(new_n229));
  oai022aa1n02x5               g134(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n230));
  nona22aa1n03x5               g135(.a(new_n223), .b(new_n229), .c(new_n230), .out0(new_n231));
  aoai13aa1n03x5               g136(.a(new_n231), .b(new_n228), .c(new_n227), .d(new_n223), .o1(\s[22] ));
  nano23aa1n03x5               g137(.a(new_n203), .b(new_n211), .c(new_n212), .d(new_n204), .out0(new_n233));
  inv000aa1d42x5               g138(.a(\a[21] ), .o1(new_n234));
  inv020aa1n04x5               g139(.a(\a[22] ), .o1(new_n235));
  xroi22aa1d06x4               g140(.a(new_n234), .b(\b[20] ), .c(new_n235), .d(\b[21] ), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  nano22aa1n02x4               g142(.a(new_n237), .b(new_n200), .c(new_n233), .out0(new_n238));
  aoai13aa1n06x5               g143(.a(new_n236), .b(new_n219), .c(new_n233), .d(new_n202), .o1(new_n239));
  oaoi03aa1n06x5               g144(.a(\a[22] ), .b(\b[21] ), .c(new_n227), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  nanp02aa1n02x5               g146(.a(new_n239), .b(new_n241), .o1(new_n242));
  xorc02aa1n12x5               g147(.a(\a[23] ), .b(\b[22] ), .out0(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n242), .c(new_n190), .d(new_n238), .o1(new_n244));
  aoi112aa1n02x5               g149(.a(new_n243), .b(new_n242), .c(new_n190), .d(new_n238), .o1(new_n245));
  norb02aa1n02x5               g150(.a(new_n244), .b(new_n245), .out0(\s[23] ));
  norp02aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  xorc02aa1n02x5               g153(.a(\a[24] ), .b(\b[23] ), .out0(new_n249));
  and002aa1n02x5               g154(.a(\b[23] ), .b(\a[24] ), .o(new_n250));
  oai022aa1n02x5               g155(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n251));
  nona22aa1n03x5               g156(.a(new_n244), .b(new_n250), .c(new_n251), .out0(new_n252));
  aoai13aa1n03x5               g157(.a(new_n252), .b(new_n249), .c(new_n248), .d(new_n244), .o1(\s[24] ));
  inv030aa1n02x5               g158(.a(new_n218), .o1(new_n254));
  and002aa1n06x5               g159(.a(new_n249), .b(new_n243), .o(new_n255));
  nano22aa1n02x4               g160(.a(new_n254), .b(new_n236), .c(new_n255), .out0(new_n256));
  inv000aa1d42x5               g161(.a(new_n255), .o1(new_n257));
  aob012aa1n02x5               g162(.a(new_n251), .b(\b[23] ), .c(\a[24] ), .out0(new_n258));
  aoai13aa1n04x5               g163(.a(new_n258), .b(new_n257), .c(new_n239), .d(new_n241), .o1(new_n259));
  xorc02aa1n12x5               g164(.a(\a[25] ), .b(\b[24] ), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n259), .c(new_n190), .d(new_n256), .o1(new_n261));
  aoi112aa1n02x5               g166(.a(new_n260), .b(new_n259), .c(new_n190), .d(new_n256), .o1(new_n262));
  norb02aa1n02x5               g167(.a(new_n261), .b(new_n262), .out0(\s[25] ));
  norp02aa1n02x5               g168(.a(\b[24] ), .b(\a[25] ), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  tech160nm_fixorc02aa1n03p5x5 g170(.a(\a[26] ), .b(\b[25] ), .out0(new_n266));
  and002aa1n02x5               g171(.a(\b[25] ), .b(\a[26] ), .o(new_n267));
  oai022aa1n02x5               g172(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n268));
  nona22aa1n03x5               g173(.a(new_n261), .b(new_n267), .c(new_n268), .out0(new_n269));
  aoai13aa1n03x5               g174(.a(new_n269), .b(new_n266), .c(new_n265), .d(new_n261), .o1(\s[26] ));
  and002aa1n06x5               g175(.a(new_n266), .b(new_n260), .o(new_n271));
  nano32aa1n03x7               g176(.a(new_n254), .b(new_n271), .c(new_n236), .d(new_n255), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n255), .b(new_n240), .c(new_n221), .d(new_n236), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n271), .o1(new_n274));
  aob012aa1n02x5               g179(.a(new_n268), .b(\b[25] ), .c(\a[26] ), .out0(new_n275));
  aoai13aa1n04x5               g180(.a(new_n275), .b(new_n274), .c(new_n273), .d(new_n258), .o1(new_n276));
  xorc02aa1n12x5               g181(.a(\a[27] ), .b(\b[26] ), .out0(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n276), .c(new_n190), .d(new_n272), .o1(new_n278));
  inv020aa1n03x5               g183(.a(new_n272), .o1(new_n279));
  aoi013aa1n06x4               g184(.a(new_n279), .b(new_n187), .c(new_n188), .d(new_n189), .o1(new_n280));
  nanb02aa1n02x5               g185(.a(new_n277), .b(new_n275), .out0(new_n281));
  aoi112aa1n03x4               g186(.a(new_n280), .b(new_n281), .c(new_n259), .d(new_n271), .o1(new_n282));
  norb02aa1n02x7               g187(.a(new_n278), .b(new_n282), .out0(\s[27] ));
  xorc02aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .out0(new_n284));
  norp02aa1n02x5               g189(.a(\b[26] ), .b(\a[27] ), .o1(new_n285));
  oaoi13aa1n03x5               g190(.a(new_n285), .b(new_n277), .c(new_n280), .d(new_n276), .o1(new_n286));
  oai112aa1n02x7               g191(.a(new_n278), .b(new_n284), .c(\b[26] ), .d(\a[27] ), .o1(new_n287));
  oaih12aa1n02x5               g192(.a(new_n287), .b(new_n286), .c(new_n284), .o1(\s[28] ));
  and002aa1n06x5               g193(.a(new_n284), .b(new_n277), .o(new_n289));
  aoai13aa1n03x5               g194(.a(new_n289), .b(new_n276), .c(new_n190), .d(new_n272), .o1(new_n290));
  nand02aa1n03x5               g195(.a(new_n190), .b(new_n272), .o1(new_n291));
  aobi12aa1n03x5               g196(.a(new_n275), .b(new_n259), .c(new_n271), .out0(new_n292));
  inv000aa1d42x5               g197(.a(new_n289), .o1(new_n293));
  orn002aa1n02x5               g198(.a(\a[27] ), .b(\b[26] ), .o(new_n294));
  oao003aa1n03x5               g199(.a(\a[28] ), .b(\b[27] ), .c(new_n294), .carry(new_n295));
  aoai13aa1n02x7               g200(.a(new_n295), .b(new_n293), .c(new_n291), .d(new_n292), .o1(new_n296));
  xorc02aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .out0(new_n297));
  norb02aa1n02x5               g202(.a(new_n295), .b(new_n297), .out0(new_n298));
  aoi022aa1n03x5               g203(.a(new_n296), .b(new_n297), .c(new_n290), .d(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g205(.a(new_n277), .b(new_n297), .c(new_n284), .o(new_n301));
  aoai13aa1n03x5               g206(.a(new_n301), .b(new_n276), .c(new_n190), .d(new_n272), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n301), .o1(new_n303));
  oaoi03aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n295), .o1(new_n304));
  inv000aa1n03x5               g209(.a(new_n304), .o1(new_n305));
  aoai13aa1n02x7               g210(.a(new_n305), .b(new_n303), .c(new_n291), .d(new_n292), .o1(new_n306));
  xorc02aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .out0(new_n307));
  norp02aa1n02x5               g212(.a(new_n304), .b(new_n307), .o1(new_n308));
  aoi022aa1n03x5               g213(.a(new_n306), .b(new_n307), .c(new_n302), .d(new_n308), .o1(\s[30] ));
  nano22aa1n06x5               g214(.a(new_n293), .b(new_n297), .c(new_n307), .out0(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n276), .c(new_n190), .d(new_n272), .o1(new_n311));
  inv000aa1d42x5               g216(.a(new_n310), .o1(new_n312));
  oao003aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n313));
  aoai13aa1n02x7               g218(.a(new_n313), .b(new_n312), .c(new_n291), .d(new_n292), .o1(new_n314));
  xorc02aa1n02x5               g219(.a(\a[31] ), .b(\b[30] ), .out0(new_n315));
  norb02aa1n02x5               g220(.a(new_n313), .b(new_n315), .out0(new_n316));
  aoi022aa1n03x5               g221(.a(new_n314), .b(new_n315), .c(new_n311), .d(new_n316), .o1(\s[31] ));
  aoi022aa1n02x5               g222(.a(new_n103), .b(new_n102), .c(\a[1] ), .d(\b[0] ), .o1(new_n318));
  oaib12aa1n02x5               g223(.a(new_n318), .b(new_n103), .c(\a[2] ), .out0(new_n319));
  norb02aa1n02x5               g224(.a(new_n109), .b(new_n108), .out0(new_n320));
  aboi22aa1n03x5               g225(.a(new_n108), .b(new_n109), .c(new_n102), .d(new_n103), .out0(new_n321));
  aboi22aa1n03x5               g226(.a(new_n105), .b(new_n320), .c(new_n321), .d(new_n319), .out0(\s[3] ));
  inv000aa1d42x5               g227(.a(new_n105), .o1(new_n323));
  obai22aa1n02x7               g228(.a(new_n107), .b(new_n106), .c(\a[3] ), .d(\b[2] ), .out0(new_n324));
  aoi012aa1n02x5               g229(.a(new_n324), .b(new_n323), .c(new_n320), .o1(new_n325));
  aoib12aa1n02x5               g230(.a(new_n325), .b(new_n114), .c(new_n106), .out0(\s[4] ));
  nanb02aa1n02x5               g231(.a(new_n110), .b(new_n323), .out0(new_n327));
  norb02aa1n02x5               g232(.a(new_n118), .b(new_n117), .out0(new_n328));
  xnbna2aa1n03x5               g233(.a(new_n328), .b(new_n327), .c(new_n113), .out0(\s[5] ));
  aoi012aa1n02x5               g234(.a(new_n117), .b(new_n114), .c(new_n118), .o1(new_n330));
  tech160nm_fiao0012aa1n02p5x5 g235(.a(new_n128), .b(new_n114), .c(new_n328), .o(new_n331));
  aoai13aa1n02x5               g236(.a(new_n331), .b(new_n330), .c(new_n127), .d(new_n116), .o1(\s[6] ));
  xnbna2aa1n03x5               g237(.a(new_n123), .b(new_n331), .c(new_n116), .out0(\s[7] ));
  aoai13aa1n02x5               g238(.a(new_n129), .b(new_n128), .c(new_n114), .d(new_n328), .o1(new_n334));
  xnbna2aa1n03x5               g239(.a(new_n125), .b(new_n334), .c(new_n131), .out0(\s[8] ));
  aoi112aa1n02x5               g240(.a(new_n133), .b(new_n135), .c(new_n114), .d(new_n124), .o1(new_n336));
  norb02aa1n02x5               g241(.a(new_n136), .b(new_n336), .out0(\s[9] ));
endmodule


