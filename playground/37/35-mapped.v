// Benchmark "adder" written by ABC on Thu Jul 18 07:14:12 2024

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
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n331,
    new_n332, new_n333, new_n336, new_n337, new_n338, new_n340, new_n341,
    new_n343;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  oaoi03aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  norp02aa1n04x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand42aa1n02x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor042aa1n04x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nona23aa1n02x4               g011(.a(new_n105), .b(new_n103), .c(new_n106), .d(new_n104), .out0(new_n107));
  inv000aa1d42x5               g012(.a(\a[3] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\b[2] ), .o1(new_n109));
  aoai13aa1n02x5               g014(.a(new_n105), .b(new_n106), .c(new_n109), .d(new_n108), .o1(new_n110));
  oai012aa1n04x7               g015(.a(new_n110), .b(new_n107), .c(new_n102), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand22aa1n03x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norp02aa1n04x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nona23aa1n02x4               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  norp02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nand02aa1n06x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nanb02aa1n02x5               g023(.a(new_n117), .b(new_n118), .out0(new_n119));
  xnrc02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .out0(new_n120));
  nor043aa1n02x5               g025(.a(new_n116), .b(new_n119), .c(new_n120), .o1(new_n121));
  inv000aa1n02x5               g026(.a(new_n113), .o1(new_n122));
  inv000aa1d42x5               g027(.a(new_n118), .o1(new_n123));
  nor002aa1n06x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(new_n124), .b(new_n115), .o1(new_n125));
  nona22aa1n03x5               g030(.a(new_n125), .b(new_n117), .c(new_n114), .out0(new_n126));
  nona22aa1n03x5               g031(.a(new_n126), .b(new_n123), .c(new_n122), .out0(new_n127));
  tech160nm_fioai012aa1n04x5   g032(.a(new_n127), .b(\b[7] ), .c(\a[8] ), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n97), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n130), .b(new_n128), .c(new_n111), .d(new_n121), .o1(new_n131));
  nor042aa1n04x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nand02aa1n06x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n131), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g040(.a(new_n132), .o1(new_n136));
  inv000aa1d42x5               g041(.a(new_n133), .o1(new_n137));
  aoai13aa1n06x5               g042(.a(new_n136), .b(new_n137), .c(new_n131), .d(new_n98), .o1(new_n138));
  xorb03aa1n02x5               g043(.a(new_n138), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nand42aa1n10x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nor022aa1n08x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand42aa1n03x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n06x4               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n140), .c(new_n138), .d(new_n141), .o1(new_n145));
  aoi112aa1n02x5               g050(.a(new_n140), .b(new_n144), .c(new_n138), .d(new_n141), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n145), .b(new_n146), .out0(\s[12] ));
  nano23aa1n03x5               g052(.a(new_n140), .b(new_n142), .c(new_n143), .d(new_n141), .out0(new_n148));
  nano23aa1n03x7               g053(.a(new_n97), .b(new_n132), .c(new_n133), .d(new_n129), .out0(new_n149));
  and002aa1n02x5               g054(.a(new_n149), .b(new_n148), .o(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n128), .c(new_n111), .d(new_n121), .o1(new_n151));
  aob012aa1n02x5               g056(.a(new_n136), .b(new_n97), .c(new_n133), .out0(new_n152));
  aoi112aa1n02x5               g057(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n153));
  aoi112aa1n03x5               g058(.a(new_n153), .b(new_n142), .c(new_n148), .d(new_n152), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(new_n151), .b(new_n154), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n04x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nand42aa1n03x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  aoi012aa1n02x5               g063(.a(new_n157), .b(new_n155), .c(new_n158), .o1(new_n159));
  xnrb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand22aa1n03x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nona23aa1n02x4               g067(.a(new_n162), .b(new_n158), .c(new_n157), .d(new_n161), .out0(new_n163));
  tech160nm_fiao0012aa1n05x5   g068(.a(new_n161), .b(new_n157), .c(new_n162), .o(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  aoai13aa1n06x5               g070(.a(new_n165), .b(new_n163), .c(new_n151), .d(new_n154), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand42aa1n06x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand42aa1n03x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n169), .b(new_n168), .out0(new_n174));
  aoi112aa1n02x5               g079(.a(new_n168), .b(new_n172), .c(new_n166), .d(new_n174), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n173), .b(new_n175), .out0(\s[16] ));
  nano23aa1n02x4               g081(.a(new_n157), .b(new_n161), .c(new_n162), .d(new_n158), .out0(new_n177));
  nano23aa1n03x5               g082(.a(new_n168), .b(new_n170), .c(new_n171), .d(new_n169), .out0(new_n178));
  nanp02aa1n02x5               g083(.a(new_n178), .b(new_n177), .o1(new_n179));
  nano22aa1n03x7               g084(.a(new_n179), .b(new_n148), .c(new_n149), .out0(new_n180));
  aoai13aa1n12x5               g085(.a(new_n180), .b(new_n128), .c(new_n111), .d(new_n121), .o1(new_n181));
  aoi112aa1n02x5               g086(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n182));
  norb02aa1n02x5               g087(.a(new_n141), .b(new_n140), .out0(new_n183));
  oai112aa1n02x5               g088(.a(new_n183), .b(new_n144), .c(new_n182), .d(new_n132), .o1(new_n184));
  nona22aa1n03x5               g089(.a(new_n184), .b(new_n153), .c(new_n142), .out0(new_n185));
  nano22aa1n02x5               g090(.a(new_n163), .b(new_n174), .c(new_n172), .out0(new_n186));
  aoi112aa1n02x5               g091(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n187));
  nand42aa1n04x5               g092(.a(new_n178), .b(new_n164), .o1(new_n188));
  nona22aa1n06x5               g093(.a(new_n188), .b(new_n187), .c(new_n170), .out0(new_n189));
  aoi012aa1d18x5               g094(.a(new_n189), .b(new_n185), .c(new_n186), .o1(new_n190));
  norp02aa1n04x5               g095(.a(\b[16] ), .b(\a[17] ), .o1(new_n191));
  nanp02aa1n09x5               g096(.a(\b[16] ), .b(\a[17] ), .o1(new_n192));
  norb02aa1n02x5               g097(.a(new_n192), .b(new_n191), .out0(new_n193));
  xnbna2aa1n03x5               g098(.a(new_n193), .b(new_n181), .c(new_n190), .out0(\s[17] ));
  nanp02aa1n09x5               g099(.a(new_n181), .b(new_n190), .o1(new_n195));
  tech160nm_fiaoi012aa1n05x5   g100(.a(new_n191), .b(new_n195), .c(new_n193), .o1(new_n196));
  xnrb03aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor002aa1n03x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  nand42aa1n10x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nano23aa1n09x5               g104(.a(new_n191), .b(new_n198), .c(new_n199), .d(new_n192), .out0(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  oa0012aa1n02x5               g106(.a(new_n199), .b(new_n198), .c(new_n191), .o(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n201), .c(new_n181), .d(new_n190), .o1(new_n204));
  xorb03aa1n02x5               g109(.a(new_n204), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1d32x5               g111(.a(\a[19] ), .o1(new_n207));
  inv000aa1d42x5               g112(.a(\b[18] ), .o1(new_n208));
  nand22aa1n06x5               g113(.a(new_n208), .b(new_n207), .o1(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  nand22aa1n03x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nor042aa1n02x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nand22aa1n02x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  aoai13aa1n03x5               g119(.a(new_n214), .b(new_n210), .c(new_n204), .d(new_n211), .o1(new_n215));
  aoi112aa1n02x5               g120(.a(new_n210), .b(new_n214), .c(new_n204), .d(new_n211), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n215), .b(new_n216), .out0(\s[20] ));
  nano22aa1n02x4               g122(.a(new_n212), .b(new_n211), .c(new_n213), .out0(new_n218));
  nand23aa1n02x5               g123(.a(new_n200), .b(new_n209), .c(new_n218), .o1(new_n219));
  nanb03aa1n09x5               g124(.a(new_n212), .b(new_n213), .c(new_n211), .out0(new_n220));
  oai022aa1n02x5               g125(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n221));
  nand03aa1n06x5               g126(.a(new_n221), .b(new_n209), .c(new_n199), .o1(new_n222));
  aoai13aa1n04x5               g127(.a(new_n213), .b(new_n212), .c(new_n207), .d(new_n208), .o1(new_n223));
  oaih12aa1n12x5               g128(.a(new_n223), .b(new_n222), .c(new_n220), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n219), .c(new_n181), .d(new_n190), .o1(new_n226));
  xorb03aa1n02x5               g131(.a(new_n226), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  xnrc02aa1n12x5               g133(.a(\b[20] ), .b(\a[21] ), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  xnrc02aa1n12x5               g135(.a(\b[21] ), .b(\a[22] ), .out0(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n03x5               g137(.a(new_n232), .b(new_n228), .c(new_n226), .d(new_n230), .o1(new_n233));
  aoi112aa1n02x5               g138(.a(new_n228), .b(new_n232), .c(new_n226), .d(new_n230), .o1(new_n234));
  norb02aa1n02x7               g139(.a(new_n233), .b(new_n234), .out0(\s[22] ));
  norp02aa1n24x5               g140(.a(new_n231), .b(new_n229), .o1(new_n236));
  nona23aa1n06x5               g141(.a(new_n236), .b(new_n200), .c(new_n210), .d(new_n220), .out0(new_n237));
  inv000aa1d42x5               g142(.a(\a[22] ), .o1(new_n238));
  inv000aa1d42x5               g143(.a(\b[21] ), .o1(new_n239));
  oaoi03aa1n02x5               g144(.a(new_n238), .b(new_n239), .c(new_n228), .o1(new_n240));
  inv000aa1n02x5               g145(.a(new_n240), .o1(new_n241));
  aoi012aa1n02x5               g146(.a(new_n241), .b(new_n224), .c(new_n236), .o1(new_n242));
  aoai13aa1n04x5               g147(.a(new_n242), .b(new_n237), .c(new_n181), .d(new_n190), .o1(new_n243));
  xorb03aa1n02x5               g148(.a(new_n243), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  xorc02aa1n02x5               g150(.a(\a[23] ), .b(\b[22] ), .out0(new_n246));
  xorc02aa1n02x5               g151(.a(\a[24] ), .b(\b[23] ), .out0(new_n247));
  aoai13aa1n02x5               g152(.a(new_n247), .b(new_n245), .c(new_n243), .d(new_n246), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(new_n245), .b(new_n247), .c(new_n243), .d(new_n246), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n248), .b(new_n249), .out0(\s[24] ));
  inv000aa1d42x5               g155(.a(new_n236), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(new_n247), .b(new_n246), .o1(new_n252));
  norp03aa1n02x5               g157(.a(new_n219), .b(new_n251), .c(new_n252), .o1(new_n253));
  oai012aa1n02x5               g158(.a(new_n199), .b(\b[18] ), .c(\a[19] ), .o1(new_n254));
  oab012aa1n02x4               g159(.a(new_n254), .b(new_n191), .c(new_n198), .out0(new_n255));
  inv000aa1n02x5               g160(.a(new_n223), .o1(new_n256));
  aoai13aa1n03x5               g161(.a(new_n236), .b(new_n256), .c(new_n255), .d(new_n218), .o1(new_n257));
  orn002aa1n02x5               g162(.a(\a[23] ), .b(\b[22] ), .o(new_n258));
  oao003aa1n02x5               g163(.a(\a[24] ), .b(\b[23] ), .c(new_n258), .carry(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n252), .c(new_n257), .d(new_n240), .o1(new_n260));
  xorc02aa1n02x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  aoai13aa1n06x5               g166(.a(new_n261), .b(new_n260), .c(new_n195), .d(new_n253), .o1(new_n262));
  aoi112aa1n02x5               g167(.a(new_n261), .b(new_n260), .c(new_n195), .d(new_n253), .o1(new_n263));
  norb02aa1n02x5               g168(.a(new_n262), .b(new_n263), .out0(\s[25] ));
  nor042aa1n03x5               g169(.a(\b[24] ), .b(\a[25] ), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  xorc02aa1n02x5               g171(.a(\a[26] ), .b(\b[25] ), .out0(new_n267));
  aobi12aa1n06x5               g172(.a(new_n267), .b(new_n262), .c(new_n266), .out0(new_n268));
  nona22aa1n06x5               g173(.a(new_n262), .b(new_n267), .c(new_n265), .out0(new_n269));
  norb02aa1n03x4               g174(.a(new_n269), .b(new_n268), .out0(\s[26] ));
  oao003aa1n02x5               g175(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n271));
  nano23aa1n02x4               g176(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n272));
  nanp02aa1n02x5               g177(.a(new_n272), .b(new_n271), .o1(new_n273));
  nano23aa1n02x4               g178(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n274));
  nona22aa1n02x4               g179(.a(new_n274), .b(new_n120), .c(new_n119), .out0(new_n275));
  aoi013aa1n02x4               g180(.a(new_n112), .b(new_n126), .c(new_n118), .d(new_n113), .o1(new_n276));
  aoai13aa1n02x5               g181(.a(new_n276), .b(new_n275), .c(new_n273), .d(new_n110), .o1(new_n277));
  inv000aa1n02x5               g182(.a(new_n189), .o1(new_n278));
  oai012aa1n03x5               g183(.a(new_n278), .b(new_n154), .c(new_n179), .o1(new_n279));
  inv000aa1n02x5               g184(.a(new_n252), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(new_n267), .b(new_n261), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n281), .o1(new_n282));
  nano22aa1n03x7               g187(.a(new_n237), .b(new_n280), .c(new_n282), .out0(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n279), .c(new_n277), .d(new_n180), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[26] ), .b(\b[25] ), .c(new_n266), .carry(new_n285));
  aobi12aa1n06x5               g190(.a(new_n285), .b(new_n260), .c(new_n282), .out0(new_n286));
  xorc02aa1n12x5               g191(.a(\a[27] ), .b(\b[26] ), .out0(new_n287));
  xnbna2aa1n03x5               g192(.a(new_n287), .b(new_n286), .c(new_n284), .out0(\s[27] ));
  nor042aa1n03x5               g193(.a(\b[26] ), .b(\a[27] ), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n289), .o1(new_n290));
  nona23aa1n12x5               g195(.a(new_n282), .b(new_n236), .c(new_n219), .d(new_n252), .out0(new_n291));
  aoi012aa1n09x5               g196(.a(new_n291), .b(new_n181), .c(new_n190), .o1(new_n292));
  aoai13aa1n06x5               g197(.a(new_n280), .b(new_n241), .c(new_n224), .d(new_n236), .o1(new_n293));
  aoai13aa1n04x5               g198(.a(new_n285), .b(new_n281), .c(new_n293), .d(new_n259), .o1(new_n294));
  tech160nm_fioai012aa1n04x5   g199(.a(new_n287), .b(new_n294), .c(new_n292), .o1(new_n295));
  xnrc02aa1n12x5               g200(.a(\b[27] ), .b(\a[28] ), .out0(new_n296));
  aoi012aa1n03x5               g201(.a(new_n296), .b(new_n295), .c(new_n290), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n287), .o1(new_n298));
  tech160nm_fiaoi012aa1n02p5x5 g203(.a(new_n298), .b(new_n286), .c(new_n284), .o1(new_n299));
  nano22aa1n02x4               g204(.a(new_n299), .b(new_n290), .c(new_n296), .out0(new_n300));
  nor002aa1n02x5               g205(.a(new_n297), .b(new_n300), .o1(\s[28] ));
  norb02aa1d21x5               g206(.a(new_n287), .b(new_n296), .out0(new_n302));
  tech160nm_fioai012aa1n05x5   g207(.a(new_n302), .b(new_n294), .c(new_n292), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[28] ), .b(\b[27] ), .c(new_n290), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[28] ), .b(\a[29] ), .out0(new_n305));
  tech160nm_fiaoi012aa1n05x5   g210(.a(new_n305), .b(new_n303), .c(new_n304), .o1(new_n306));
  inv000aa1d42x5               g211(.a(new_n302), .o1(new_n307));
  tech160nm_fiaoi012aa1n02p5x5 g212(.a(new_n307), .b(new_n286), .c(new_n284), .o1(new_n308));
  nano22aa1n02x4               g213(.a(new_n308), .b(new_n304), .c(new_n305), .out0(new_n309));
  nor002aa1n02x5               g214(.a(new_n306), .b(new_n309), .o1(\s[29] ));
  xorb03aa1n02x5               g215(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n03x5               g216(.a(new_n287), .b(new_n305), .c(new_n296), .out0(new_n312));
  oaih12aa1n02x5               g217(.a(new_n312), .b(new_n294), .c(new_n292), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[29] ), .b(\b[28] ), .c(new_n304), .carry(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[29] ), .b(\a[30] ), .out0(new_n315));
  tech160nm_fiaoi012aa1n02p5x5 g220(.a(new_n315), .b(new_n313), .c(new_n314), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n312), .o1(new_n317));
  aoi012aa1n02x5               g222(.a(new_n317), .b(new_n286), .c(new_n284), .o1(new_n318));
  nano22aa1n02x4               g223(.a(new_n318), .b(new_n314), .c(new_n315), .out0(new_n319));
  norp02aa1n03x5               g224(.a(new_n316), .b(new_n319), .o1(\s[30] ));
  norb02aa1n02x5               g225(.a(new_n312), .b(new_n315), .out0(new_n321));
  inv000aa1n02x5               g226(.a(new_n321), .o1(new_n322));
  tech160nm_fiaoi012aa1n04x5   g227(.a(new_n322), .b(new_n286), .c(new_n284), .o1(new_n323));
  oao003aa1n02x5               g228(.a(\a[30] ), .b(\b[29] ), .c(new_n314), .carry(new_n324));
  xnrc02aa1n02x5               g229(.a(\b[30] ), .b(\a[31] ), .out0(new_n325));
  nano22aa1n03x7               g230(.a(new_n323), .b(new_n324), .c(new_n325), .out0(new_n326));
  tech160nm_fioai012aa1n04x5   g231(.a(new_n321), .b(new_n294), .c(new_n292), .o1(new_n327));
  tech160nm_fiaoi012aa1n02p5x5 g232(.a(new_n325), .b(new_n327), .c(new_n324), .o1(new_n328));
  norp02aa1n02x5               g233(.a(new_n328), .b(new_n326), .o1(\s[31] ));
  xorb03aa1n02x5               g234(.a(new_n102), .b(\b[2] ), .c(new_n108), .out0(\s[3] ));
  norb02aa1n02x5               g235(.a(new_n105), .b(new_n106), .out0(new_n331));
  oaoi03aa1n02x5               g236(.a(new_n108), .b(new_n109), .c(new_n271), .o1(new_n332));
  oabi12aa1n02x5               g237(.a(new_n104), .b(new_n107), .c(new_n102), .out0(new_n333));
  mtn022aa1n02x5               g238(.a(new_n333), .b(new_n332), .sa(new_n331), .o1(\s[4] ));
  xorb03aa1n02x5               g239(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g240(.a(new_n114), .b(new_n115), .out0(new_n336));
  inv000aa1d42x5               g241(.a(new_n124), .o1(new_n337));
  nanb02aa1n02x5               g242(.a(new_n120), .b(new_n111), .out0(new_n338));
  xobna2aa1n03x5               g243(.a(new_n336), .b(new_n338), .c(new_n337), .out0(\s[6] ));
  aoi012aa1n02x5               g244(.a(new_n114), .b(new_n124), .c(new_n115), .o1(new_n340));
  aoai13aa1n02x5               g245(.a(new_n340), .b(new_n336), .c(new_n338), .d(new_n337), .o1(new_n341));
  xorb03aa1n02x5               g246(.a(new_n341), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g247(.a(new_n117), .b(new_n341), .c(new_n118), .o1(new_n343));
  xnrb03aa1n02x5               g248(.a(new_n343), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g249(.a(new_n277), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


