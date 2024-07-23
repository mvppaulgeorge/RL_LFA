// Benchmark "adder" written by ABC on Thu Jul 18 08:23:12 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n321, new_n323, new_n324, new_n325, new_n327;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n04x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n06x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor042aa1n12x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\a[2] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\b[1] ), .o1(new_n103));
  nand02aa1n06x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  tech160nm_fioaoi03aa1n04x5   g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  nor022aa1n06x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nanp02aa1n04x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nor022aa1n16x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nand42aa1n03x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nona23aa1n09x5               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  oai012aa1n02x5               g015(.a(new_n107), .b(new_n108), .c(new_n106), .o1(new_n111));
  oai012aa1n06x5               g016(.a(new_n111), .b(new_n110), .c(new_n105), .o1(new_n112));
  norp02aa1n12x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nand02aa1d06x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nanb02aa1n02x5               g019(.a(new_n113), .b(new_n114), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nor022aa1n08x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nand42aa1n04x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nor022aa1n16x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nand42aa1n04x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nona23aa1n09x5               g025(.a(new_n120), .b(new_n118), .c(new_n117), .d(new_n119), .out0(new_n121));
  nor043aa1n04x5               g026(.a(new_n121), .b(new_n116), .c(new_n115), .o1(new_n122));
  nano23aa1n09x5               g027(.a(new_n117), .b(new_n119), .c(new_n120), .d(new_n118), .out0(new_n123));
  nor042aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  tech160nm_fiao0012aa1n02p5x5 g029(.a(new_n113), .b(new_n124), .c(new_n114), .o(new_n125));
  oai012aa1n02x5               g030(.a(new_n118), .b(new_n119), .c(new_n117), .o1(new_n126));
  aob012aa1n06x5               g031(.a(new_n126), .b(new_n123), .c(new_n125), .out0(new_n127));
  xorc02aa1n12x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n127), .c(new_n112), .d(new_n122), .o1(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n101), .out0(\s[10] ));
  oao003aa1n02x5               g035(.a(new_n102), .b(new_n103), .c(new_n104), .carry(new_n131));
  nano23aa1n03x5               g036(.a(new_n106), .b(new_n108), .c(new_n109), .d(new_n107), .out0(new_n132));
  aobi12aa1n06x5               g037(.a(new_n111), .b(new_n132), .c(new_n131), .out0(new_n133));
  nona22aa1n02x4               g038(.a(new_n123), .b(new_n116), .c(new_n115), .out0(new_n134));
  aobi12aa1n06x5               g039(.a(new_n126), .b(new_n123), .c(new_n125), .out0(new_n135));
  tech160nm_fioai012aa1n03p5x5 g040(.a(new_n135), .b(new_n133), .c(new_n134), .o1(new_n136));
  aoai13aa1n03x5               g041(.a(new_n99), .b(new_n100), .c(new_n136), .d(new_n128), .o1(new_n137));
  tech160nm_fioai012aa1n04x5   g042(.a(new_n137), .b(\b[9] ), .c(\a[10] ), .o1(new_n138));
  xorb03aa1n02x5               g043(.a(new_n138), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nanp02aa1n02x5               g044(.a(new_n129), .b(new_n101), .o1(new_n140));
  nor002aa1n03x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nand42aa1n06x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  norb02aa1n03x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n97), .c(new_n140), .d(new_n98), .o1(new_n144));
  nor042aa1n06x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nanp02aa1n12x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  norb02aa1d21x5               g051(.a(new_n146), .b(new_n145), .out0(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  oaoi13aa1n02x5               g053(.a(new_n148), .b(new_n144), .c(\a[11] ), .d(\b[10] ), .o1(new_n149));
  aoi112aa1n02x5               g054(.a(new_n147), .b(new_n141), .c(new_n138), .d(new_n143), .o1(new_n150));
  norp02aa1n03x5               g055(.a(new_n149), .b(new_n150), .o1(\s[12] ));
  nanp02aa1n02x5               g056(.a(new_n112), .b(new_n122), .o1(new_n152));
  nano23aa1n06x5               g057(.a(new_n141), .b(new_n145), .c(new_n146), .d(new_n142), .out0(new_n153));
  nand23aa1n03x5               g058(.a(new_n153), .b(new_n99), .c(new_n128), .o1(new_n154));
  aoi012aa1n03x5               g059(.a(new_n97), .b(new_n100), .c(new_n98), .o1(new_n155));
  nanb03aa1n06x5               g060(.a(new_n155), .b(new_n143), .c(new_n147), .out0(new_n156));
  oai012aa1n02x7               g061(.a(new_n146), .b(new_n145), .c(new_n141), .o1(new_n157));
  and002aa1n02x5               g062(.a(new_n156), .b(new_n157), .o(new_n158));
  aoai13aa1n06x5               g063(.a(new_n158), .b(new_n154), .c(new_n152), .d(new_n135), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g065(.a(\a[14] ), .o1(new_n161));
  nor042aa1n09x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  tech160nm_fixorc02aa1n05x5   g067(.a(\a[13] ), .b(\b[12] ), .out0(new_n163));
  aoi012aa1n02x5               g068(.a(new_n162), .b(new_n159), .c(new_n163), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[13] ), .c(new_n161), .out0(\s[14] ));
  xorc02aa1n06x5               g070(.a(\a[14] ), .b(\b[13] ), .out0(new_n166));
  and002aa1n02x5               g071(.a(new_n166), .b(new_n163), .o(new_n167));
  inv000aa1d42x5               g072(.a(\b[13] ), .o1(new_n168));
  oaoi03aa1n12x5               g073(.a(new_n161), .b(new_n168), .c(new_n162), .o1(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  nor042aa1n09x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanp02aa1n04x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  aoai13aa1n06x5               g078(.a(new_n173), .b(new_n170), .c(new_n159), .d(new_n167), .o1(new_n174));
  aoi112aa1n02x5               g079(.a(new_n173), .b(new_n170), .c(new_n159), .d(new_n167), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(\s[15] ));
  inv000aa1d42x5               g081(.a(new_n171), .o1(new_n177));
  nor042aa1n02x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanp02aa1n04x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  aobi12aa1n02x5               g085(.a(new_n180), .b(new_n174), .c(new_n177), .out0(new_n181));
  nona22aa1n02x4               g086(.a(new_n174), .b(new_n180), .c(new_n171), .out0(new_n182));
  norb02aa1n02x5               g087(.a(new_n182), .b(new_n181), .out0(\s[16] ));
  nano23aa1n06x5               g088(.a(new_n171), .b(new_n178), .c(new_n179), .d(new_n172), .out0(new_n184));
  nand23aa1n06x5               g089(.a(new_n184), .b(new_n163), .c(new_n166), .o1(new_n185));
  nor042aa1n06x5               g090(.a(new_n185), .b(new_n154), .o1(new_n186));
  aoai13aa1n12x5               g091(.a(new_n186), .b(new_n127), .c(new_n112), .d(new_n122), .o1(new_n187));
  tech160nm_fiaoi012aa1n04x5   g092(.a(new_n185), .b(new_n156), .c(new_n157), .o1(new_n188));
  oai022aa1n02x5               g093(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n189));
  aboi22aa1n03x5               g094(.a(new_n169), .b(new_n184), .c(new_n189), .d(new_n179), .out0(new_n190));
  norb02aa1n09x5               g095(.a(new_n190), .b(new_n188), .out0(new_n191));
  nand02aa1d06x5               g096(.a(new_n187), .b(new_n191), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g098(.a(\a[18] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\a[17] ), .o1(new_n195));
  inv000aa1d42x5               g100(.a(\b[16] ), .o1(new_n196));
  oaoi03aa1n02x5               g101(.a(new_n195), .b(new_n196), .c(new_n192), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[17] ), .c(new_n194), .out0(\s[18] ));
  xroi22aa1d06x4               g103(.a(new_n195), .b(\b[16] ), .c(new_n194), .d(\b[17] ), .out0(new_n199));
  nor002aa1n04x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  aoi112aa1n09x5               g105(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n201));
  nor042aa1n06x5               g106(.a(new_n201), .b(new_n200), .o1(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  nor002aa1n16x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nanp02aa1n09x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  norb02aa1n06x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  aoai13aa1n06x5               g111(.a(new_n206), .b(new_n203), .c(new_n192), .d(new_n199), .o1(new_n207));
  aoi112aa1n02x5               g112(.a(new_n206), .b(new_n203), .c(new_n192), .d(new_n199), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1d18x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nand42aa1d28x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  norb02aa1n15x5               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  oaoi13aa1n06x5               g119(.a(new_n214), .b(new_n207), .c(\a[19] ), .d(\b[18] ), .o1(new_n215));
  nona22aa1n02x5               g120(.a(new_n207), .b(new_n213), .c(new_n204), .out0(new_n216));
  norb02aa1n03x4               g121(.a(new_n216), .b(new_n215), .out0(\s[20] ));
  nona23aa1n09x5               g122(.a(new_n212), .b(new_n205), .c(new_n204), .d(new_n211), .out0(new_n218));
  inv040aa1n03x5               g123(.a(new_n218), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(new_n199), .b(new_n219), .o1(new_n220));
  oai012aa1n04x7               g125(.a(new_n212), .b(new_n211), .c(new_n204), .o1(new_n221));
  oai012aa1n18x5               g126(.a(new_n221), .b(new_n218), .c(new_n202), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoai13aa1n04x5               g128(.a(new_n223), .b(new_n220), .c(new_n187), .d(new_n191), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor022aa1n16x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  xorc02aa1n12x5               g131(.a(\a[21] ), .b(\b[20] ), .out0(new_n227));
  xorc02aa1n12x5               g132(.a(\a[22] ), .b(\b[21] ), .out0(new_n228));
  aoai13aa1n03x5               g133(.a(new_n228), .b(new_n226), .c(new_n224), .d(new_n227), .o1(new_n229));
  aoi112aa1n02x5               g134(.a(new_n226), .b(new_n228), .c(new_n224), .d(new_n227), .o1(new_n230));
  norb02aa1n02x5               g135(.a(new_n229), .b(new_n230), .out0(\s[22] ));
  nand02aa1n10x5               g136(.a(new_n228), .b(new_n227), .o1(new_n232));
  nanb03aa1n06x5               g137(.a(new_n232), .b(new_n199), .c(new_n219), .out0(new_n233));
  oai112aa1n06x5               g138(.a(new_n206), .b(new_n213), .c(new_n201), .d(new_n200), .o1(new_n234));
  inv000aa1d42x5               g139(.a(\a[22] ), .o1(new_n235));
  inv040aa1d32x5               g140(.a(\b[21] ), .o1(new_n236));
  oaoi03aa1n12x5               g141(.a(new_n235), .b(new_n236), .c(new_n226), .o1(new_n237));
  aoai13aa1n12x5               g142(.a(new_n237), .b(new_n232), .c(new_n234), .d(new_n221), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  aoai13aa1n06x5               g144(.a(new_n239), .b(new_n233), .c(new_n187), .d(new_n191), .o1(new_n240));
  xorb03aa1n02x5               g145(.a(new_n240), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  tech160nm_fixorc02aa1n02p5x5 g147(.a(\a[23] ), .b(\b[22] ), .out0(new_n243));
  xorc02aa1n02x5               g148(.a(\a[24] ), .b(\b[23] ), .out0(new_n244));
  aoai13aa1n03x5               g149(.a(new_n244), .b(new_n242), .c(new_n240), .d(new_n243), .o1(new_n245));
  aoi112aa1n02x5               g150(.a(new_n242), .b(new_n244), .c(new_n240), .d(new_n243), .o1(new_n246));
  norb02aa1n02x7               g151(.a(new_n245), .b(new_n246), .out0(\s[24] ));
  and002aa1n02x5               g152(.a(new_n244), .b(new_n243), .o(new_n248));
  nona23aa1n02x4               g153(.a(new_n248), .b(new_n199), .c(new_n232), .d(new_n218), .out0(new_n249));
  inv000aa1d42x5               g154(.a(\a[24] ), .o1(new_n250));
  inv000aa1d42x5               g155(.a(\b[23] ), .o1(new_n251));
  oao003aa1n02x5               g156(.a(new_n250), .b(new_n251), .c(new_n242), .carry(new_n252));
  tech160nm_fiaoi012aa1n05x5   g157(.a(new_n252), .b(new_n238), .c(new_n248), .o1(new_n253));
  aoai13aa1n06x5               g158(.a(new_n253), .b(new_n249), .c(new_n187), .d(new_n191), .o1(new_n254));
  xorb03aa1n02x5               g159(.a(new_n254), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  tech160nm_fixorc02aa1n05x5   g161(.a(\a[25] ), .b(\b[24] ), .out0(new_n257));
  tech160nm_fixorc02aa1n02p5x5 g162(.a(\a[26] ), .b(\b[25] ), .out0(new_n258));
  aoai13aa1n03x5               g163(.a(new_n258), .b(new_n256), .c(new_n254), .d(new_n257), .o1(new_n259));
  aoi112aa1n02x5               g164(.a(new_n256), .b(new_n258), .c(new_n254), .d(new_n257), .o1(new_n260));
  norb02aa1n03x4               g165(.a(new_n259), .b(new_n260), .out0(\s[26] ));
  aoai13aa1n02x5               g166(.a(new_n190), .b(new_n185), .c(new_n156), .d(new_n157), .o1(new_n262));
  and002aa1n06x5               g167(.a(new_n258), .b(new_n257), .o(new_n263));
  nano22aa1n03x7               g168(.a(new_n233), .b(new_n248), .c(new_n263), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n262), .c(new_n136), .d(new_n186), .o1(new_n265));
  aoai13aa1n09x5               g170(.a(new_n263), .b(new_n252), .c(new_n238), .d(new_n248), .o1(new_n266));
  oai022aa1n02x5               g171(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n267));
  aob012aa1n02x5               g172(.a(new_n267), .b(\b[25] ), .c(\a[26] ), .out0(new_n268));
  xorc02aa1n12x5               g173(.a(\a[27] ), .b(\b[26] ), .out0(new_n269));
  inv000aa1d42x5               g174(.a(new_n269), .o1(new_n270));
  aoi013aa1n06x4               g175(.a(new_n270), .b(new_n265), .c(new_n266), .d(new_n268), .o1(new_n271));
  inv020aa1n03x5               g176(.a(new_n264), .o1(new_n272));
  aoi012aa1n09x5               g177(.a(new_n272), .b(new_n187), .c(new_n191), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n232), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n237), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n248), .b(new_n275), .c(new_n222), .d(new_n274), .o1(new_n276));
  inv000aa1n02x5               g181(.a(new_n252), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n263), .o1(new_n278));
  aoai13aa1n06x5               g183(.a(new_n268), .b(new_n278), .c(new_n276), .d(new_n277), .o1(new_n279));
  norp03aa1n02x5               g184(.a(new_n279), .b(new_n273), .c(new_n269), .o1(new_n280));
  norp02aa1n02x5               g185(.a(new_n271), .b(new_n280), .o1(\s[27] ));
  norp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  inv040aa1n03x5               g187(.a(new_n282), .o1(new_n283));
  oaih12aa1n02x5               g188(.a(new_n269), .b(new_n279), .c(new_n273), .o1(new_n284));
  xnrc02aa1n12x5               g189(.a(\b[27] ), .b(\a[28] ), .out0(new_n285));
  tech160nm_fiaoi012aa1n02p5x5 g190(.a(new_n285), .b(new_n284), .c(new_n283), .o1(new_n286));
  nano22aa1n03x5               g191(.a(new_n271), .b(new_n283), .c(new_n285), .out0(new_n287));
  norp02aa1n03x5               g192(.a(new_n286), .b(new_n287), .o1(\s[28] ));
  norb02aa1d21x5               g193(.a(new_n269), .b(new_n285), .out0(new_n289));
  oaih12aa1n02x5               g194(.a(new_n289), .b(new_n279), .c(new_n273), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[28] ), .b(\a[29] ), .out0(new_n292));
  tech160nm_fiaoi012aa1n02p5x5 g197(.a(new_n292), .b(new_n290), .c(new_n291), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n289), .o1(new_n294));
  aoi013aa1n02x5               g199(.a(new_n294), .b(new_n265), .c(new_n266), .d(new_n268), .o1(new_n295));
  nano22aa1n03x5               g200(.a(new_n295), .b(new_n291), .c(new_n292), .out0(new_n296));
  nor002aa1n02x5               g201(.a(new_n293), .b(new_n296), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g203(.a(new_n269), .b(new_n292), .c(new_n285), .out0(new_n299));
  oai012aa1n02x5               g204(.a(new_n299), .b(new_n279), .c(new_n273), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[29] ), .b(\a[30] ), .out0(new_n302));
  aoi012aa1n02x7               g207(.a(new_n302), .b(new_n300), .c(new_n301), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n299), .o1(new_n304));
  aoi013aa1n02x5               g209(.a(new_n304), .b(new_n265), .c(new_n266), .d(new_n268), .o1(new_n305));
  nano22aa1n03x5               g210(.a(new_n305), .b(new_n301), .c(new_n302), .out0(new_n306));
  norp02aa1n03x5               g211(.a(new_n303), .b(new_n306), .o1(\s[30] ));
  norb02aa1n02x5               g212(.a(new_n299), .b(new_n302), .out0(new_n308));
  oaih12aa1n02x5               g213(.a(new_n308), .b(new_n279), .c(new_n273), .o1(new_n309));
  oao003aa1n02x5               g214(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[30] ), .b(\a[31] ), .out0(new_n311));
  tech160nm_fiaoi012aa1n02p5x5 g216(.a(new_n311), .b(new_n309), .c(new_n310), .o1(new_n312));
  inv000aa1n02x5               g217(.a(new_n308), .o1(new_n313));
  aoi013aa1n02x5               g218(.a(new_n313), .b(new_n265), .c(new_n266), .d(new_n268), .o1(new_n314));
  nano22aa1n03x5               g219(.a(new_n314), .b(new_n310), .c(new_n311), .out0(new_n315));
  norp02aa1n03x5               g220(.a(new_n312), .b(new_n315), .o1(\s[31] ));
  xnrb03aa1n02x5               g221(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g222(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g224(.a(new_n112), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  tech160nm_fioaoi03aa1n03p5x5 g225(.a(\a[5] ), .b(\b[4] ), .c(new_n133), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g227(.a(new_n120), .b(new_n119), .out0(new_n323));
  aoai13aa1n02x5               g228(.a(new_n323), .b(new_n113), .c(new_n321), .d(new_n114), .o1(new_n324));
  aoi112aa1n02x5               g229(.a(new_n323), .b(new_n113), .c(new_n321), .d(new_n114), .o1(new_n325));
  norb02aa1n02x5               g230(.a(new_n324), .b(new_n325), .out0(\s[7] ));
  tech160nm_fioai012aa1n03p5x5 g231(.a(new_n324), .b(\b[6] ), .c(\a[7] ), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g233(.a(new_n128), .b(new_n152), .c(new_n135), .out0(\s[9] ));
endmodule


