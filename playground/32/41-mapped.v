// Benchmark "adder" written by ABC on Thu Jul 18 04:44:06 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n238, new_n239, new_n240, new_n241, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n260, new_n261, new_n262, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n317, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n336, new_n337, new_n339,
    new_n341, new_n343, new_n345;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[2] ), .b(\a[3] ), .o1(new_n97));
  nand02aa1n04x5               g002(.a(\b[2] ), .b(\a[3] ), .o1(new_n98));
  nanb02aa1n03x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nanp02aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  oai112aa1n06x5               g005(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n101));
  nanp02aa1n06x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  tech160nm_finor002aa1n03p5x5 g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nor002aa1n02x5               g008(.a(new_n103), .b(new_n97), .o1(new_n104));
  oai012aa1n09x5               g009(.a(new_n104), .b(new_n102), .c(new_n99), .o1(new_n105));
  inv000aa1d42x5               g010(.a(\a[7] ), .o1(new_n106));
  inv000aa1d42x5               g011(.a(\b[6] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(new_n107), .b(new_n106), .o1(new_n108));
  nand02aa1n06x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  oai112aa1n02x5               g014(.a(new_n108), .b(new_n109), .c(\b[7] ), .d(\a[8] ), .o1(new_n110));
  nand02aa1n06x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nor022aa1n12x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  aoi012aa1n02x7               g017(.a(new_n112), .b(\a[8] ), .c(\b[7] ), .o1(new_n113));
  nand42aa1n06x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nor002aa1d32x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nand42aa1n06x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nanb03aa1n03x5               g021(.a(new_n115), .b(new_n116), .c(new_n114), .out0(new_n117));
  nano23aa1n03x7               g022(.a(new_n110), .b(new_n117), .c(new_n113), .d(new_n111), .out0(new_n118));
  nor022aa1n08x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  nor022aa1n04x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nor042aa1n02x5               g025(.a(new_n120), .b(new_n119), .o1(new_n121));
  oai112aa1n06x5               g026(.a(new_n109), .b(new_n116), .c(new_n115), .d(new_n112), .o1(new_n122));
  aoi022aa1n09x5               g027(.a(new_n122), .b(new_n121), .c(\a[8] ), .d(\b[7] ), .o1(new_n123));
  aoi012aa1n12x5               g028(.a(new_n123), .b(new_n118), .c(new_n105), .o1(new_n124));
  oaoi03aa1n02x5               g029(.a(\a[9] ), .b(\b[8] ), .c(new_n124), .o1(new_n125));
  xorb03aa1n02x5               g030(.a(new_n125), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  aoi113aa1n03x7               g031(.a(new_n97), .b(new_n103), .c(new_n101), .d(new_n100), .e(new_n98), .o1(new_n127));
  norb03aa1n03x5               g032(.a(new_n109), .b(new_n119), .c(new_n120), .out0(new_n128));
  inv000aa1n02x5               g033(.a(new_n111), .o1(new_n129));
  nand42aa1n04x5               g034(.a(\b[7] ), .b(\a[8] ), .o1(new_n130));
  oai012aa1n03x5               g035(.a(new_n130), .b(\b[5] ), .c(\a[6] ), .o1(new_n131));
  nano22aa1n03x7               g036(.a(new_n115), .b(new_n114), .c(new_n116), .out0(new_n132));
  nona23aa1n06x5               g037(.a(new_n132), .b(new_n128), .c(new_n129), .d(new_n131), .out0(new_n133));
  aob012aa1n02x5               g038(.a(new_n130), .b(new_n122), .c(new_n121), .out0(new_n134));
  oai012aa1n12x5               g039(.a(new_n134), .b(new_n127), .c(new_n133), .o1(new_n135));
  nor002aa1d32x5               g040(.a(\b[8] ), .b(\a[9] ), .o1(new_n136));
  nand42aa1d28x5               g041(.a(\b[8] ), .b(\a[9] ), .o1(new_n137));
  norb02aa1n03x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  nor022aa1n16x5               g043(.a(\b[9] ), .b(\a[10] ), .o1(new_n139));
  nand02aa1d28x5               g044(.a(\b[9] ), .b(\a[10] ), .o1(new_n140));
  norb02aa1n03x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  aoi012aa1d24x5               g046(.a(new_n139), .b(new_n136), .c(new_n140), .o1(new_n142));
  inv000aa1d42x5               g047(.a(new_n142), .o1(new_n143));
  aoi013aa1n06x4               g048(.a(new_n143), .b(new_n135), .c(new_n138), .d(new_n141), .o1(new_n144));
  nor042aa1n09x5               g049(.a(\b[10] ), .b(\a[11] ), .o1(new_n145));
  inv000aa1n02x5               g050(.a(new_n145), .o1(new_n146));
  nanp02aa1n04x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  xnbna2aa1n03x5               g052(.a(new_n144), .b(new_n147), .c(new_n146), .out0(\s[11] ));
  oaoi03aa1n03x5               g053(.a(\a[11] ), .b(\b[10] ), .c(new_n144), .o1(new_n149));
  xorb03aa1n02x5               g054(.a(new_n149), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor022aa1n08x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  tech160nm_finand02aa1n05x5   g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nanb03aa1n09x5               g057(.a(new_n151), .b(new_n152), .c(new_n147), .out0(new_n153));
  nano32aa1n03x7               g058(.a(new_n153), .b(new_n141), .c(new_n138), .d(new_n146), .out0(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n123), .c(new_n118), .d(new_n105), .o1(new_n155));
  aoi012aa1n06x5               g060(.a(new_n151), .b(new_n145), .c(new_n152), .o1(new_n156));
  oai013aa1n03x5               g061(.a(new_n156), .b(new_n153), .c(new_n142), .d(new_n145), .o1(new_n157));
  inv000aa1n02x5               g062(.a(new_n157), .o1(new_n158));
  xorc02aa1n12x5               g063(.a(\a[13] ), .b(\b[12] ), .out0(new_n159));
  xnbna2aa1n03x5               g064(.a(new_n159), .b(new_n155), .c(new_n158), .out0(\s[13] ));
  xorc02aa1n12x5               g065(.a(\a[14] ), .b(\b[13] ), .out0(new_n161));
  nanp02aa1n02x5               g066(.a(new_n155), .b(new_n158), .o1(new_n162));
  nor042aa1n02x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  aoi012aa1n02x5               g068(.a(new_n163), .b(new_n162), .c(new_n159), .o1(new_n164));
  xnrc02aa1n02x5               g069(.a(new_n164), .b(new_n161), .out0(\s[14] ));
  inv000aa1d42x5               g070(.a(new_n159), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n161), .o1(new_n167));
  nona22aa1n02x4               g072(.a(new_n162), .b(new_n166), .c(new_n167), .out0(new_n168));
  nor042aa1n04x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  aob012aa1n06x5               g074(.a(new_n163), .b(\b[13] ), .c(\a[14] ), .out0(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  nor002aa1d32x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nand42aa1d28x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n168), .c(new_n171), .out0(\s[15] ));
  nanp02aa1n02x5               g080(.a(new_n168), .b(new_n171), .o1(new_n176));
  nor042aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand42aa1n06x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  aoi112aa1n02x5               g084(.a(new_n172), .b(new_n179), .c(new_n176), .d(new_n173), .o1(new_n180));
  inv000aa1d42x5               g085(.a(new_n172), .o1(new_n181));
  aoi112aa1n03x5               g086(.a(new_n167), .b(new_n166), .c(new_n155), .d(new_n158), .o1(new_n182));
  oaib12aa1n02x5               g087(.a(new_n174), .b(new_n182), .c(new_n171), .out0(new_n183));
  aobi12aa1n06x5               g088(.a(new_n179), .b(new_n183), .c(new_n181), .out0(new_n184));
  nor002aa1n02x5               g089(.a(new_n184), .b(new_n180), .o1(\s[16] ));
  nano23aa1n02x4               g090(.a(new_n136), .b(new_n139), .c(new_n140), .d(new_n137), .out0(new_n186));
  nano22aa1n03x7               g091(.a(new_n151), .b(new_n147), .c(new_n152), .out0(new_n187));
  nano23aa1n03x7               g092(.a(new_n172), .b(new_n177), .c(new_n178), .d(new_n173), .out0(new_n188));
  nand23aa1n04x5               g093(.a(new_n188), .b(new_n159), .c(new_n161), .o1(new_n189));
  nano32aa1n03x7               g094(.a(new_n189), .b(new_n187), .c(new_n186), .d(new_n146), .out0(new_n190));
  nanb03aa1n03x5               g095(.a(new_n142), .b(new_n187), .c(new_n146), .out0(new_n191));
  nona22aa1n02x4               g096(.a(new_n170), .b(new_n172), .c(new_n169), .out0(new_n192));
  aoi022aa1n02x5               g097(.a(\b[15] ), .b(\a[16] ), .c(\a[15] ), .d(\b[14] ), .o1(new_n193));
  aoi012aa1n09x5               g098(.a(new_n177), .b(new_n192), .c(new_n193), .o1(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n189), .c(new_n191), .d(new_n156), .o1(new_n195));
  aoi012aa1n12x5               g100(.a(new_n195), .b(new_n135), .c(new_n190), .o1(new_n196));
  tech160nm_fixorc02aa1n03p5x5 g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  xnrc02aa1n02x5               g102(.a(new_n196), .b(new_n197), .out0(\s[17] ));
  oaoi03aa1n02x5               g103(.a(\a[17] ), .b(\b[16] ), .c(new_n196), .o1(new_n199));
  xorb03aa1n02x5               g104(.a(new_n199), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor042aa1n04x5               g105(.a(\b[16] ), .b(\a[17] ), .o1(new_n201));
  nor042aa1n06x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nand22aa1n09x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  oa0012aa1n02x5               g108(.a(new_n203), .b(new_n202), .c(new_n201), .o(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  nanb02aa1n12x5               g110(.a(new_n189), .b(new_n154), .out0(new_n206));
  and003aa1n03x7               g111(.a(new_n188), .b(new_n161), .c(new_n159), .o(new_n207));
  aobi12aa1n12x5               g112(.a(new_n194), .b(new_n207), .c(new_n157), .out0(new_n208));
  oaih12aa1n12x5               g113(.a(new_n208), .b(new_n206), .c(new_n124), .o1(new_n209));
  norb02aa1n03x5               g114(.a(new_n203), .b(new_n202), .out0(new_n210));
  nand23aa1n03x5               g115(.a(new_n209), .b(new_n197), .c(new_n210), .o1(new_n211));
  nor002aa1d32x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nand42aa1n06x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n211), .c(new_n205), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1n03x5               g121(.a(new_n212), .o1(new_n217));
  inv000aa1n02x5               g122(.a(new_n214), .o1(new_n218));
  tech160nm_fiaoi012aa1n05x5   g123(.a(new_n218), .b(new_n211), .c(new_n205), .o1(new_n219));
  nor002aa1n16x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  nand22aa1n12x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nanb02aa1n02x5               g126(.a(new_n220), .b(new_n221), .out0(new_n222));
  nano22aa1n03x7               g127(.a(new_n219), .b(new_n217), .c(new_n222), .out0(new_n223));
  nano22aa1n03x7               g128(.a(new_n196), .b(new_n197), .c(new_n210), .out0(new_n224));
  oai012aa1n02x7               g129(.a(new_n214), .b(new_n224), .c(new_n204), .o1(new_n225));
  aoi012aa1n03x5               g130(.a(new_n222), .b(new_n225), .c(new_n217), .o1(new_n226));
  nor002aa1n02x5               g131(.a(new_n226), .b(new_n223), .o1(\s[20] ));
  nanb03aa1n12x5               g132(.a(new_n220), .b(new_n221), .c(new_n213), .out0(new_n228));
  nano32aa1n03x7               g133(.a(new_n228), .b(new_n197), .c(new_n210), .d(new_n217), .out0(new_n229));
  aoai13aa1n03x5               g134(.a(new_n229), .b(new_n195), .c(new_n135), .d(new_n190), .o1(new_n230));
  oai112aa1n06x5               g135(.a(new_n217), .b(new_n203), .c(new_n202), .d(new_n201), .o1(new_n231));
  aoi012aa1d24x5               g136(.a(new_n220), .b(new_n212), .c(new_n221), .o1(new_n232));
  oai012aa1d24x5               g137(.a(new_n232), .b(new_n231), .c(new_n228), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  xnrc02aa1n12x5               g139(.a(\b[20] ), .b(\a[21] ), .out0(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  xnbna2aa1n03x5               g141(.a(new_n236), .b(new_n230), .c(new_n234), .out0(\s[21] ));
  aoai13aa1n06x5               g142(.a(new_n236), .b(new_n233), .c(new_n209), .d(new_n229), .o1(new_n238));
  tech160nm_fixnrc02aa1n04x5   g143(.a(\b[21] ), .b(\a[22] ), .out0(new_n239));
  oai112aa1n03x5               g144(.a(new_n238), .b(new_n239), .c(\b[20] ), .d(\a[21] ), .o1(new_n240));
  oaoi13aa1n02x7               g145(.a(new_n239), .b(new_n238), .c(\a[21] ), .d(\b[20] ), .o1(new_n241));
  norb02aa1n03x4               g146(.a(new_n240), .b(new_n241), .out0(\s[22] ));
  nor042aa1n06x5               g147(.a(new_n239), .b(new_n235), .o1(new_n243));
  and002aa1n02x5               g148(.a(new_n229), .b(new_n243), .o(new_n244));
  nano22aa1n02x5               g149(.a(new_n220), .b(new_n213), .c(new_n221), .out0(new_n245));
  oai012aa1n02x5               g150(.a(new_n203), .b(\b[18] ), .c(\a[19] ), .o1(new_n246));
  oab012aa1n02x5               g151(.a(new_n246), .b(new_n201), .c(new_n202), .out0(new_n247));
  inv020aa1n02x5               g152(.a(new_n232), .o1(new_n248));
  aoai13aa1n06x5               g153(.a(new_n243), .b(new_n248), .c(new_n247), .d(new_n245), .o1(new_n249));
  inv000aa1d42x5               g154(.a(\a[22] ), .o1(new_n250));
  inv000aa1d42x5               g155(.a(\b[21] ), .o1(new_n251));
  nor042aa1n03x5               g156(.a(\b[20] ), .b(\a[21] ), .o1(new_n252));
  oaoi03aa1n12x5               g157(.a(new_n250), .b(new_n251), .c(new_n252), .o1(new_n253));
  nanp02aa1n02x5               g158(.a(new_n249), .b(new_n253), .o1(new_n254));
  xnrc02aa1n12x5               g159(.a(\b[22] ), .b(\a[23] ), .out0(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n254), .c(new_n209), .d(new_n244), .o1(new_n257));
  aoi112aa1n02x5               g162(.a(new_n256), .b(new_n254), .c(new_n209), .d(new_n244), .o1(new_n258));
  norb02aa1n02x5               g163(.a(new_n257), .b(new_n258), .out0(\s[23] ));
  tech160nm_fixnrc02aa1n05x5   g164(.a(\b[23] ), .b(\a[24] ), .out0(new_n260));
  oai112aa1n03x5               g165(.a(new_n257), .b(new_n260), .c(\b[22] ), .d(\a[23] ), .o1(new_n261));
  oaoi13aa1n06x5               g166(.a(new_n260), .b(new_n257), .c(\a[23] ), .d(\b[22] ), .o1(new_n262));
  norb02aa1n03x4               g167(.a(new_n261), .b(new_n262), .out0(\s[24] ));
  nor042aa1n03x5               g168(.a(new_n260), .b(new_n255), .o1(new_n264));
  inv030aa1n02x5               g169(.a(new_n264), .o1(new_n265));
  norp02aa1n02x5               g170(.a(\b[23] ), .b(\a[24] ), .o1(new_n266));
  aoi112aa1n02x5               g171(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n267));
  nor042aa1n02x5               g172(.a(new_n267), .b(new_n266), .o1(new_n268));
  aoai13aa1n12x5               g173(.a(new_n268), .b(new_n265), .c(new_n249), .d(new_n253), .o1(new_n269));
  inv030aa1n02x5               g174(.a(new_n269), .o1(new_n270));
  and002aa1n06x5               g175(.a(new_n264), .b(new_n243), .o(new_n271));
  inv000aa1n02x5               g176(.a(new_n271), .o1(new_n272));
  xnrc02aa1n12x5               g177(.a(\b[24] ), .b(\a[25] ), .out0(new_n273));
  oaoi13aa1n02x7               g178(.a(new_n273), .b(new_n270), .c(new_n230), .d(new_n272), .o1(new_n274));
  nano22aa1n03x7               g179(.a(new_n196), .b(new_n229), .c(new_n271), .out0(new_n275));
  nano22aa1n02x4               g180(.a(new_n275), .b(new_n270), .c(new_n273), .out0(new_n276));
  norp02aa1n02x5               g181(.a(new_n274), .b(new_n276), .o1(\s[25] ));
  nor042aa1n03x5               g182(.a(\b[24] ), .b(\a[25] ), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n278), .o1(new_n279));
  tech160nm_fixnrc02aa1n02p5x5 g184(.a(\b[25] ), .b(\a[26] ), .out0(new_n280));
  nano22aa1n03x5               g185(.a(new_n274), .b(new_n279), .c(new_n280), .out0(new_n281));
  inv000aa1d42x5               g186(.a(new_n273), .o1(new_n282));
  oaih12aa1n02x5               g187(.a(new_n282), .b(new_n275), .c(new_n269), .o1(new_n283));
  aoi012aa1n03x5               g188(.a(new_n280), .b(new_n283), .c(new_n279), .o1(new_n284));
  nor002aa1n02x5               g189(.a(new_n284), .b(new_n281), .o1(\s[26] ));
  nor042aa1n03x5               g190(.a(new_n280), .b(new_n273), .o1(new_n286));
  oao003aa1n02x5               g191(.a(\a[26] ), .b(\b[25] ), .c(new_n279), .carry(new_n287));
  aobi12aa1n12x5               g192(.a(new_n287), .b(new_n269), .c(new_n286), .out0(new_n288));
  inv000aa1n02x5               g193(.a(new_n229), .o1(new_n289));
  inv000aa1n02x5               g194(.a(new_n286), .o1(new_n290));
  nona32aa1n09x5               g195(.a(new_n209), .b(new_n290), .c(new_n272), .d(new_n289), .out0(new_n291));
  nor042aa1n04x5               g196(.a(\b[26] ), .b(\a[27] ), .o1(new_n292));
  and002aa1n12x5               g197(.a(\b[26] ), .b(\a[27] ), .o(new_n293));
  norp02aa1n02x5               g198(.a(new_n293), .b(new_n292), .o1(new_n294));
  xnbna2aa1n03x5               g199(.a(new_n294), .b(new_n291), .c(new_n288), .out0(\s[27] ));
  inv000aa1d42x5               g200(.a(new_n293), .o1(new_n296));
  xorc02aa1n02x5               g201(.a(\a[28] ), .b(\b[27] ), .out0(new_n297));
  inv000aa1n02x5               g202(.a(new_n253), .o1(new_n298));
  aoai13aa1n03x5               g203(.a(new_n264), .b(new_n298), .c(new_n233), .d(new_n243), .o1(new_n299));
  aoai13aa1n04x5               g204(.a(new_n287), .b(new_n290), .c(new_n299), .d(new_n268), .o1(new_n300));
  nano22aa1n02x4               g205(.a(new_n290), .b(new_n243), .c(new_n264), .out0(new_n301));
  nano22aa1n03x7               g206(.a(new_n196), .b(new_n229), .c(new_n301), .out0(new_n302));
  norp03aa1n03x5               g207(.a(new_n302), .b(new_n300), .c(new_n292), .o1(new_n303));
  nano22aa1n03x5               g208(.a(new_n303), .b(new_n296), .c(new_n297), .out0(new_n304));
  inv000aa1d42x5               g209(.a(new_n292), .o1(new_n305));
  nanp03aa1n03x5               g210(.a(new_n291), .b(new_n288), .c(new_n305), .o1(new_n306));
  aoi012aa1n03x5               g211(.a(new_n297), .b(new_n306), .c(new_n296), .o1(new_n307));
  nor002aa1n02x5               g212(.a(new_n307), .b(new_n304), .o1(\s[28] ));
  and002aa1n02x5               g213(.a(new_n297), .b(new_n294), .o(new_n309));
  oai012aa1n03x5               g214(.a(new_n309), .b(new_n302), .c(new_n300), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[28] ), .b(\b[27] ), .c(new_n305), .carry(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[28] ), .b(\a[29] ), .out0(new_n312));
  aoi012aa1n02x7               g217(.a(new_n312), .b(new_n310), .c(new_n311), .o1(new_n313));
  aobi12aa1n02x7               g218(.a(new_n309), .b(new_n291), .c(new_n288), .out0(new_n314));
  nano22aa1n03x5               g219(.a(new_n314), .b(new_n311), .c(new_n312), .out0(new_n315));
  norp02aa1n03x5               g220(.a(new_n313), .b(new_n315), .o1(\s[29] ));
  nanp02aa1n02x5               g221(.a(\b[0] ), .b(\a[1] ), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g223(.a(new_n312), .b(new_n297), .c(new_n294), .out0(new_n319));
  oai012aa1n03x5               g224(.a(new_n319), .b(new_n302), .c(new_n300), .o1(new_n320));
  oao003aa1n02x5               g225(.a(\a[29] ), .b(\b[28] ), .c(new_n311), .carry(new_n321));
  xnrc02aa1n02x5               g226(.a(\b[29] ), .b(\a[30] ), .out0(new_n322));
  aoi012aa1n03x5               g227(.a(new_n322), .b(new_n320), .c(new_n321), .o1(new_n323));
  aobi12aa1n06x5               g228(.a(new_n319), .b(new_n291), .c(new_n288), .out0(new_n324));
  nano22aa1n03x5               g229(.a(new_n324), .b(new_n321), .c(new_n322), .out0(new_n325));
  nor002aa1n02x5               g230(.a(new_n323), .b(new_n325), .o1(\s[30] ));
  nano23aa1n02x4               g231(.a(new_n322), .b(new_n312), .c(new_n297), .d(new_n294), .out0(new_n327));
  oai012aa1n03x5               g232(.a(new_n327), .b(new_n302), .c(new_n300), .o1(new_n328));
  oao003aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .c(new_n321), .carry(new_n329));
  xnrc02aa1n02x5               g234(.a(\b[30] ), .b(\a[31] ), .out0(new_n330));
  tech160nm_fiaoi012aa1n02p5x5 g235(.a(new_n330), .b(new_n328), .c(new_n329), .o1(new_n331));
  aobi12aa1n02x7               g236(.a(new_n327), .b(new_n291), .c(new_n288), .out0(new_n332));
  nano22aa1n03x5               g237(.a(new_n332), .b(new_n329), .c(new_n330), .out0(new_n333));
  nor002aa1n02x5               g238(.a(new_n331), .b(new_n333), .o1(\s[31] ));
  xnbna2aa1n03x5               g239(.a(new_n99), .b(new_n101), .c(new_n100), .out0(\s[3] ));
  norb02aa1n02x5               g240(.a(new_n111), .b(new_n103), .out0(new_n336));
  aoi113aa1n02x5               g241(.a(new_n97), .b(new_n336), .c(new_n101), .d(new_n100), .e(new_n98), .o1(new_n337));
  aoi012aa1n02x5               g242(.a(new_n337), .b(new_n105), .c(new_n336), .o1(\s[4] ));
  nanb02aa1n02x5               g243(.a(new_n115), .b(new_n114), .out0(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n339), .b(new_n105), .c(new_n111), .out0(\s[5] ));
  aoai13aa1n03x5               g245(.a(new_n114), .b(new_n115), .c(new_n105), .d(new_n111), .o1(new_n341));
  xnrb03aa1n02x5               g246(.a(new_n341), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n03x5               g247(.a(\a[6] ), .b(\b[5] ), .c(new_n341), .o1(new_n343));
  xorb03aa1n02x5               g248(.a(new_n343), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n03x5               g249(.a(new_n106), .b(new_n107), .c(new_n343), .o1(new_n345));
  xnrb03aa1n03x5               g250(.a(new_n345), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g251(.a(new_n135), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule

