// Benchmark "adder" written by ABC on Thu Jul 18 00:10:01 2024

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
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n319, new_n320, new_n322, new_n323, new_n326,
    new_n328, new_n329, new_n331;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d28x5               g001(.a(\a[9] ), .o1(new_n97));
  inv040aa1d32x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  and002aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o(new_n100));
  nanp02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nand42aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nor002aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  norb03aa1n03x5               g008(.a(new_n102), .b(new_n101), .c(new_n103), .out0(new_n104));
  inv040aa1d32x5               g009(.a(\a[3] ), .o1(new_n105));
  inv000aa1d42x5               g010(.a(\b[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(new_n106), .b(new_n105), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nand03aa1n02x5               g013(.a(new_n107), .b(new_n102), .c(new_n108), .o1(new_n109));
  oa0022aa1n02x5               g014(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n110));
  oaoi13aa1n09x5               g015(.a(new_n100), .b(new_n110), .c(new_n104), .d(new_n109), .o1(new_n111));
  inv000aa1d42x5               g016(.a(\a[6] ), .o1(new_n112));
  inv000aa1d42x5               g017(.a(\b[5] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nor022aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nona23aa1n02x4               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  norp02aa1n02x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nand02aa1n03x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  nanb03aa1n02x5               g026(.a(new_n119), .b(new_n121), .c(new_n120), .out0(new_n122));
  aoi112aa1n03x5               g027(.a(new_n118), .b(new_n122), .c(new_n112), .d(new_n113), .o1(new_n123));
  aoi112aa1n02x5               g028(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n124));
  norp02aa1n02x5               g029(.a(\b[5] ), .b(\a[6] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n117), .b(new_n116), .out0(new_n126));
  nano22aa1n02x4               g031(.a(new_n119), .b(new_n120), .c(new_n121), .out0(new_n127));
  oai112aa1n02x5               g032(.a(new_n127), .b(new_n126), .c(new_n114), .d(new_n125), .o1(new_n128));
  nona22aa1n02x4               g033(.a(new_n128), .b(new_n124), .c(new_n119), .out0(new_n129));
  tech160nm_fixorc02aa1n04x5   g034(.a(\a[9] ), .b(\b[8] ), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n130), .b(new_n129), .c(new_n111), .d(new_n123), .o1(new_n131));
  nor002aa1d32x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nand42aa1d28x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  norb02aa1n09x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n131), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g040(.a(new_n134), .o1(new_n136));
  tech160nm_fiaoi012aa1n05x5   g041(.a(new_n136), .b(new_n131), .c(new_n99), .o1(new_n137));
  aoai13aa1n12x5               g042(.a(new_n133), .b(new_n132), .c(new_n97), .d(new_n98), .o1(new_n138));
  inv000aa1d42x5               g043(.a(new_n138), .o1(new_n139));
  nor042aa1n09x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nand42aa1n02x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nanb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(new_n142));
  oab012aa1n02x4               g047(.a(new_n142), .b(new_n137), .c(new_n139), .out0(new_n143));
  nano22aa1n02x4               g048(.a(new_n137), .b(new_n138), .c(new_n142), .out0(new_n144));
  norp02aa1n02x5               g049(.a(new_n143), .b(new_n144), .o1(\s[11] ));
  inv000aa1d42x5               g050(.a(new_n140), .o1(new_n146));
  oabi12aa1n02x5               g051(.a(new_n142), .b(new_n137), .c(new_n139), .out0(new_n147));
  nor042aa1n02x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nand02aa1n03x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nanb02aa1n02x5               g054(.a(new_n148), .b(new_n149), .out0(new_n150));
  aoi012aa1n02x5               g055(.a(new_n150), .b(new_n147), .c(new_n146), .o1(new_n151));
  nano22aa1n02x4               g056(.a(new_n143), .b(new_n146), .c(new_n150), .out0(new_n152));
  norp02aa1n02x5               g057(.a(new_n152), .b(new_n151), .o1(\s[12] ));
  nano23aa1n06x5               g058(.a(new_n140), .b(new_n148), .c(new_n149), .d(new_n141), .out0(new_n154));
  nand23aa1d12x5               g059(.a(new_n154), .b(new_n130), .c(new_n134), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n129), .c(new_n111), .d(new_n123), .o1(new_n157));
  nona23aa1n09x5               g062(.a(new_n149), .b(new_n141), .c(new_n140), .d(new_n148), .out0(new_n158));
  nanp02aa1n02x5               g063(.a(new_n140), .b(new_n149), .o1(new_n159));
  oai122aa1n12x5               g064(.a(new_n159), .b(new_n158), .c(new_n138), .d(\b[11] ), .e(\a[12] ), .o1(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  xorc02aa1n02x5               g066(.a(\a[13] ), .b(\b[12] ), .out0(new_n162));
  xnbna2aa1n03x5               g067(.a(new_n162), .b(new_n157), .c(new_n161), .out0(\s[13] ));
  inv000aa1d42x5               g068(.a(\a[13] ), .o1(new_n164));
  inv000aa1d42x5               g069(.a(\b[12] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(new_n165), .b(new_n164), .o1(new_n166));
  oai012aa1n02x5               g071(.a(new_n110), .b(new_n104), .c(new_n109), .o1(new_n167));
  norp02aa1n02x5               g072(.a(new_n118), .b(new_n122), .o1(new_n168));
  nona23aa1n02x4               g073(.a(new_n167), .b(new_n168), .c(new_n125), .d(new_n100), .out0(new_n169));
  nanb02aa1n02x5               g074(.a(new_n129), .b(new_n169), .out0(new_n170));
  aoai13aa1n02x5               g075(.a(new_n162), .b(new_n160), .c(new_n170), .d(new_n156), .o1(new_n171));
  xnrc02aa1n02x5               g076(.a(\b[13] ), .b(\a[14] ), .out0(new_n172));
  xobna2aa1n03x5               g077(.a(new_n172), .b(new_n171), .c(new_n166), .out0(\s[14] ));
  inv020aa1n04x5               g078(.a(\a[14] ), .o1(new_n174));
  xroi22aa1d06x4               g079(.a(new_n164), .b(\b[12] ), .c(new_n174), .d(\b[13] ), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  oao003aa1n02x5               g081(.a(\a[14] ), .b(\b[13] ), .c(new_n166), .carry(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n176), .c(new_n157), .d(new_n161), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  xnrc02aa1n12x5               g085(.a(\b[14] ), .b(\a[15] ), .out0(new_n181));
  inv000aa1d42x5               g086(.a(new_n181), .o1(new_n182));
  xorc02aa1n02x5               g087(.a(\a[16] ), .b(\b[15] ), .out0(new_n183));
  aoai13aa1n02x5               g088(.a(new_n183), .b(new_n180), .c(new_n178), .d(new_n182), .o1(new_n184));
  aoi112aa1n02x5               g089(.a(new_n180), .b(new_n183), .c(new_n178), .d(new_n182), .o1(new_n185));
  norb02aa1n02x5               g090(.a(new_n184), .b(new_n185), .out0(\s[16] ));
  xnrc02aa1n02x5               g091(.a(\b[15] ), .b(\a[16] ), .out0(new_n187));
  norp02aa1n02x5               g092(.a(new_n187), .b(new_n181), .o1(new_n188));
  nano22aa1n03x7               g093(.a(new_n155), .b(new_n175), .c(new_n188), .out0(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n129), .c(new_n111), .d(new_n123), .o1(new_n190));
  nano23aa1n03x7               g095(.a(new_n181), .b(new_n172), .c(new_n183), .d(new_n162), .out0(new_n191));
  aoi112aa1n02x5               g096(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n192));
  orn002aa1n02x5               g097(.a(\a[16] ), .b(\b[15] ), .o(new_n193));
  oai013aa1n02x4               g098(.a(new_n193), .b(new_n177), .c(new_n181), .d(new_n187), .o1(new_n194));
  aoi112aa1n09x5               g099(.a(new_n194), .b(new_n192), .c(new_n160), .d(new_n191), .o1(new_n195));
  xorc02aa1n02x5               g100(.a(\a[17] ), .b(\b[16] ), .out0(new_n196));
  xnbna2aa1n03x5               g101(.a(new_n196), .b(new_n195), .c(new_n190), .out0(\s[17] ));
  inv000aa1d42x5               g102(.a(\a[17] ), .o1(new_n198));
  inv000aa1d42x5               g103(.a(\b[16] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(new_n199), .b(new_n198), .o1(new_n200));
  norp02aa1n02x5               g105(.a(\b[15] ), .b(\a[16] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(new_n160), .b(new_n191), .o1(new_n202));
  norp03aa1n02x5               g107(.a(new_n177), .b(new_n181), .c(new_n187), .o1(new_n203));
  nona32aa1n02x4               g108(.a(new_n202), .b(new_n203), .c(new_n192), .d(new_n201), .out0(new_n204));
  aoai13aa1n02x5               g109(.a(new_n196), .b(new_n204), .c(new_n170), .d(new_n189), .o1(new_n205));
  nor042aa1n02x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nand42aa1n04x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  nanb02aa1n06x5               g112(.a(new_n206), .b(new_n207), .out0(new_n208));
  xobna2aa1n03x5               g113(.a(new_n208), .b(new_n205), .c(new_n200), .out0(\s[18] ));
  nanp02aa1n02x5               g114(.a(\b[16] ), .b(\a[17] ), .o1(new_n210));
  nano22aa1n09x5               g115(.a(new_n208), .b(new_n200), .c(new_n210), .out0(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  aoai13aa1n12x5               g117(.a(new_n207), .b(new_n206), .c(new_n198), .d(new_n199), .o1(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n212), .c(new_n195), .d(new_n190), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n04x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  nanp02aa1n02x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  norp02aa1n04x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  aoai13aa1n03x5               g126(.a(new_n221), .b(new_n217), .c(new_n214), .d(new_n218), .o1(new_n222));
  aoi112aa1n02x7               g127(.a(new_n217), .b(new_n221), .c(new_n214), .d(new_n218), .o1(new_n223));
  norb02aa1n03x4               g128(.a(new_n222), .b(new_n223), .out0(\s[20] ));
  nano23aa1n06x5               g129(.a(new_n217), .b(new_n219), .c(new_n220), .d(new_n218), .out0(new_n225));
  nanp02aa1n02x5               g130(.a(new_n211), .b(new_n225), .o1(new_n226));
  nona23aa1n02x4               g131(.a(new_n220), .b(new_n218), .c(new_n217), .d(new_n219), .out0(new_n227));
  aoi012aa1n02x7               g132(.a(new_n219), .b(new_n217), .c(new_n220), .o1(new_n228));
  oai012aa1n12x5               g133(.a(new_n228), .b(new_n227), .c(new_n213), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n226), .c(new_n195), .d(new_n190), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[21] ), .b(\b[20] ), .out0(new_n234));
  xorc02aa1n02x5               g139(.a(\a[22] ), .b(\b[21] ), .out0(new_n235));
  aoai13aa1n03x5               g140(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n236));
  aoi112aa1n02x7               g141(.a(new_n233), .b(new_n235), .c(new_n231), .d(new_n234), .o1(new_n237));
  norb02aa1n03x4               g142(.a(new_n236), .b(new_n237), .out0(\s[22] ));
  inv000aa1d42x5               g143(.a(\a[21] ), .o1(new_n239));
  inv000aa1d42x5               g144(.a(\a[22] ), .o1(new_n240));
  xroi22aa1d04x5               g145(.a(new_n239), .b(\b[20] ), .c(new_n240), .d(\b[21] ), .out0(new_n241));
  nand23aa1n02x5               g146(.a(new_n241), .b(new_n211), .c(new_n225), .o1(new_n242));
  inv000aa1d42x5               g147(.a(\b[21] ), .o1(new_n243));
  oaoi03aa1n12x5               g148(.a(new_n240), .b(new_n243), .c(new_n233), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  aoi012aa1n02x5               g150(.a(new_n245), .b(new_n229), .c(new_n241), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n242), .c(new_n195), .d(new_n190), .o1(new_n247));
  xorb03aa1n02x5               g152(.a(new_n247), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  xorc02aa1n02x5               g154(.a(\a[23] ), .b(\b[22] ), .out0(new_n250));
  xorc02aa1n02x5               g155(.a(\a[24] ), .b(\b[23] ), .out0(new_n251));
  aoai13aa1n03x5               g156(.a(new_n251), .b(new_n249), .c(new_n247), .d(new_n250), .o1(new_n252));
  aoi112aa1n02x7               g157(.a(new_n249), .b(new_n251), .c(new_n247), .d(new_n250), .o1(new_n253));
  norb02aa1n03x4               g158(.a(new_n252), .b(new_n253), .out0(\s[24] ));
  and002aa1n02x5               g159(.a(new_n251), .b(new_n250), .o(new_n255));
  inv000aa1n02x5               g160(.a(new_n255), .o1(new_n256));
  nano32aa1n02x4               g161(.a(new_n256), .b(new_n241), .c(new_n225), .d(new_n211), .out0(new_n257));
  inv000aa1n02x5               g162(.a(new_n257), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n213), .o1(new_n259));
  inv000aa1n02x5               g164(.a(new_n228), .o1(new_n260));
  aoai13aa1n03x5               g165(.a(new_n241), .b(new_n260), .c(new_n225), .d(new_n259), .o1(new_n261));
  aoi112aa1n02x5               g166(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n262));
  oab012aa1n02x4               g167(.a(new_n262), .b(\a[24] ), .c(\b[23] ), .out0(new_n263));
  aoai13aa1n04x5               g168(.a(new_n263), .b(new_n256), .c(new_n261), .d(new_n244), .o1(new_n264));
  inv040aa1n03x5               g169(.a(new_n264), .o1(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n258), .c(new_n195), .d(new_n190), .o1(new_n266));
  xorb03aa1n02x5               g171(.a(new_n266), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  tech160nm_fixorc02aa1n03p5x5 g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  xorc02aa1n06x5               g174(.a(\a[26] ), .b(\b[25] ), .out0(new_n270));
  aoai13aa1n03x5               g175(.a(new_n270), .b(new_n268), .c(new_n266), .d(new_n269), .o1(new_n271));
  aoi112aa1n02x7               g176(.a(new_n268), .b(new_n270), .c(new_n266), .d(new_n269), .o1(new_n272));
  norb02aa1n03x4               g177(.a(new_n271), .b(new_n272), .out0(\s[26] ));
  and002aa1n02x5               g178(.a(new_n270), .b(new_n269), .o(new_n274));
  nano22aa1d15x5               g179(.a(new_n242), .b(new_n255), .c(new_n274), .out0(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n204), .c(new_n170), .d(new_n189), .o1(new_n276));
  aoi112aa1n02x5               g181(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n277));
  oab012aa1n02x4               g182(.a(new_n277), .b(\a[26] ), .c(\b[25] ), .out0(new_n278));
  aobi12aa1n06x5               g183(.a(new_n278), .b(new_n264), .c(new_n274), .out0(new_n279));
  xorc02aa1n02x5               g184(.a(\a[27] ), .b(\b[26] ), .out0(new_n280));
  xnbna2aa1n03x5               g185(.a(new_n280), .b(new_n276), .c(new_n279), .out0(\s[27] ));
  norp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  inv040aa1n03x5               g187(.a(new_n282), .o1(new_n283));
  nanp02aa1n06x5               g188(.a(new_n195), .b(new_n190), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n255), .b(new_n245), .c(new_n229), .d(new_n241), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n274), .o1(new_n286));
  aoai13aa1n06x5               g191(.a(new_n278), .b(new_n286), .c(new_n285), .d(new_n263), .o1(new_n287));
  aoai13aa1n06x5               g192(.a(new_n280), .b(new_n287), .c(new_n284), .d(new_n275), .o1(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[27] ), .b(\a[28] ), .out0(new_n289));
  tech160nm_fiaoi012aa1n05x5   g194(.a(new_n289), .b(new_n288), .c(new_n283), .o1(new_n290));
  aobi12aa1n02x7               g195(.a(new_n280), .b(new_n276), .c(new_n279), .out0(new_n291));
  nano22aa1n02x4               g196(.a(new_n291), .b(new_n283), .c(new_n289), .out0(new_n292));
  nor002aa1n02x5               g197(.a(new_n290), .b(new_n292), .o1(\s[28] ));
  norb02aa1n02x5               g198(.a(new_n280), .b(new_n289), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n287), .c(new_n284), .d(new_n275), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[28] ), .b(\a[29] ), .out0(new_n297));
  aoi012aa1n03x5               g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n02x7               g203(.a(new_n294), .b(new_n276), .c(new_n279), .out0(new_n299));
  nano22aa1n02x4               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  nor002aa1n02x5               g205(.a(new_n298), .b(new_n300), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g207(.a(new_n280), .b(new_n297), .c(new_n289), .out0(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n287), .c(new_n284), .d(new_n275), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[29] ), .b(\a[30] ), .out0(new_n306));
  aoi012aa1n03x5               g211(.a(new_n306), .b(new_n304), .c(new_n305), .o1(new_n307));
  aobi12aa1n02x5               g212(.a(new_n303), .b(new_n276), .c(new_n279), .out0(new_n308));
  nano22aa1n02x4               g213(.a(new_n308), .b(new_n305), .c(new_n306), .out0(new_n309));
  nor002aa1n02x5               g214(.a(new_n307), .b(new_n309), .o1(\s[30] ));
  xnrc02aa1n02x5               g215(.a(\b[30] ), .b(\a[31] ), .out0(new_n311));
  norb02aa1n02x5               g216(.a(new_n303), .b(new_n306), .out0(new_n312));
  aobi12aa1n02x7               g217(.a(new_n312), .b(new_n276), .c(new_n279), .out0(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n314));
  nano22aa1n02x4               g219(.a(new_n313), .b(new_n311), .c(new_n314), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n312), .b(new_n287), .c(new_n284), .d(new_n275), .o1(new_n316));
  aoi012aa1n03x5               g221(.a(new_n311), .b(new_n316), .c(new_n314), .o1(new_n317));
  nor002aa1n02x5               g222(.a(new_n317), .b(new_n315), .o1(\s[31] ));
  norp02aa1n02x5               g223(.a(new_n104), .b(new_n109), .o1(new_n319));
  aboi22aa1n03x5               g224(.a(new_n104), .b(new_n102), .c(new_n108), .d(new_n107), .out0(new_n320));
  norp02aa1n02x5               g225(.a(new_n320), .b(new_n319), .o1(\s[3] ));
  xnrc02aa1n02x5               g226(.a(\b[3] ), .b(\a[4] ), .out0(new_n322));
  nano22aa1n02x4               g227(.a(new_n319), .b(new_n107), .c(new_n322), .out0(new_n323));
  oaoi13aa1n02x5               g228(.a(new_n323), .b(new_n111), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xorb03aa1n02x5               g229(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  tech160nm_fiao0012aa1n02p5x5 g230(.a(new_n114), .b(new_n111), .c(new_n115), .o(new_n326));
  xorb03aa1n02x5               g231(.a(new_n326), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi13aa1n02x5               g232(.a(new_n126), .b(new_n121), .c(new_n326), .d(new_n125), .o1(new_n328));
  oai112aa1n02x5               g233(.a(new_n121), .b(new_n126), .c(new_n326), .d(new_n125), .o1(new_n329));
  norb02aa1n02x5               g234(.a(new_n329), .b(new_n328), .out0(\s[7] ));
  oai012aa1n02x5               g235(.a(new_n329), .b(\b[6] ), .c(\a[7] ), .o1(new_n331));
  xorb03aa1n02x5               g236(.a(new_n331), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g237(.a(new_n170), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


