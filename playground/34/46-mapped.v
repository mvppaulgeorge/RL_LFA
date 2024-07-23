// Benchmark "adder" written by ABC on Thu Jul 18 05:48:40 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n213, new_n214, new_n215, new_n216, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n324, new_n325, new_n326, new_n328, new_n329, new_n331, new_n332,
    new_n334, new_n335, new_n337, new_n338, new_n340, new_n341, new_n342,
    new_n344, new_n345, new_n346;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand02aa1n16x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nor002aa1d24x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nand02aa1d28x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nor042aa1d18x5               g005(.a(\b[8] ), .b(\a[9] ), .o1(new_n101));
  nand02aa1d10x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  nor002aa1d32x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  nor002aa1d32x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nand02aa1n04x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nona23aa1d18x5               g010(.a(new_n102), .b(new_n105), .c(new_n104), .d(new_n103), .out0(new_n106));
  inv000aa1d42x5               g011(.a(\a[6] ), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\b[5] ), .o1(new_n108));
  nor042aa1n04x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  oaoi03aa1n12x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  norp02aa1n02x5               g015(.a(new_n106), .b(new_n110), .o1(new_n111));
  nor002aa1n04x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  nand22aa1n03x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  nand02aa1d06x5               g018(.a(\b[0] ), .b(\a[1] ), .o1(new_n114));
  aoi012aa1n12x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  nor002aa1d32x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nand22aa1n12x5               g021(.a(\b[3] ), .b(\a[4] ), .o1(new_n117));
  nor002aa1d32x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nand02aa1n04x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  nona23aa1d18x5               g024(.a(new_n119), .b(new_n117), .c(new_n116), .d(new_n118), .out0(new_n120));
  aoi012aa1d24x5               g025(.a(new_n116), .b(new_n118), .c(new_n117), .o1(new_n121));
  oai012aa1n18x5               g026(.a(new_n121), .b(new_n120), .c(new_n115), .o1(new_n122));
  tech160nm_fixnrc02aa1n04x5   g027(.a(\b[5] ), .b(\a[6] ), .out0(new_n123));
  xnrc02aa1n12x5               g028(.a(\b[4] ), .b(\a[5] ), .out0(new_n124));
  nor043aa1n06x5               g029(.a(new_n106), .b(new_n123), .c(new_n124), .o1(new_n125));
  nanp02aa1n03x5               g030(.a(new_n122), .b(new_n125), .o1(new_n126));
  oa0012aa1n03x5               g031(.a(new_n102), .b(new_n103), .c(new_n104), .o(new_n127));
  nona32aa1n03x5               g032(.a(new_n126), .b(new_n111), .c(new_n127), .d(new_n101), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n99), .b(new_n128), .c(new_n100), .out0(\s[10] ));
  nanb03aa1n06x5               g034(.a(new_n99), .b(new_n128), .c(new_n100), .out0(new_n130));
  oa0012aa1n02x5               g035(.a(new_n97), .b(new_n98), .c(new_n101), .o(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  nor042aa1d18x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand42aa1d28x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n06x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n130), .c(new_n132), .out0(\s[11] ));
  inv000aa1d42x5               g041(.a(new_n133), .o1(new_n137));
  aob012aa1n03x5               g042(.a(new_n135), .b(new_n130), .c(new_n132), .out0(new_n138));
  nor042aa1n06x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand42aa1d28x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  inv000aa1d42x5               g046(.a(new_n135), .o1(new_n142));
  norb03aa1n02x5               g047(.a(new_n140), .b(new_n133), .c(new_n139), .out0(new_n143));
  aoai13aa1n02x7               g048(.a(new_n143), .b(new_n142), .c(new_n130), .d(new_n132), .o1(new_n144));
  aoai13aa1n03x5               g049(.a(new_n144), .b(new_n141), .c(new_n138), .d(new_n137), .o1(\s[12] ));
  oabi12aa1n18x5               g050(.a(new_n127), .b(new_n106), .c(new_n110), .out0(new_n146));
  nano23aa1n09x5               g051(.a(new_n133), .b(new_n139), .c(new_n140), .d(new_n134), .out0(new_n147));
  nano23aa1n09x5               g052(.a(new_n101), .b(new_n98), .c(new_n97), .d(new_n100), .out0(new_n148));
  nand22aa1n09x5               g053(.a(new_n148), .b(new_n147), .o1(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  aoai13aa1n06x5               g055(.a(new_n150), .b(new_n146), .c(new_n122), .d(new_n125), .o1(new_n151));
  oai112aa1n02x5               g056(.a(new_n97), .b(new_n134), .c(new_n101), .d(new_n98), .o1(new_n152));
  nona22aa1n03x5               g057(.a(new_n152), .b(new_n139), .c(new_n133), .out0(new_n153));
  and002aa1n02x5               g058(.a(new_n153), .b(new_n140), .o(new_n154));
  inv000aa1n02x5               g059(.a(new_n154), .o1(new_n155));
  nor022aa1n12x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand42aa1d28x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n158), .b(new_n151), .c(new_n155), .out0(\s[13] ));
  inv000aa1n02x5               g064(.a(new_n146), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(new_n126), .b(new_n160), .o1(new_n161));
  aoai13aa1n02x5               g066(.a(new_n158), .b(new_n154), .c(new_n161), .d(new_n150), .o1(new_n162));
  nor002aa1d32x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  inv000aa1d42x5               g068(.a(\a[13] ), .o1(new_n164));
  inv000aa1d42x5               g069(.a(\b[12] ), .o1(new_n165));
  nand42aa1d28x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  aboi22aa1n03x5               g071(.a(new_n163), .b(new_n166), .c(new_n164), .d(new_n165), .out0(new_n167));
  nano23aa1d15x5               g072(.a(new_n156), .b(new_n163), .c(new_n166), .d(new_n157), .out0(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n166), .b(new_n163), .c(new_n164), .d(new_n165), .o1(new_n170));
  aoai13aa1n06x5               g075(.a(new_n170), .b(new_n169), .c(new_n151), .d(new_n155), .o1(new_n171));
  aboi22aa1n03x5               g076(.a(new_n163), .b(new_n171), .c(new_n162), .d(new_n167), .out0(\s[14] ));
  aoai13aa1n06x5               g077(.a(new_n168), .b(new_n154), .c(new_n161), .d(new_n150), .o1(new_n173));
  nor042aa1n06x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nand02aa1d12x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nanb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(new_n176));
  xobna2aa1n03x5               g081(.a(new_n176), .b(new_n173), .c(new_n170), .out0(\s[15] ));
  nor002aa1n12x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nand22aa1n12x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanb02aa1n02x5               g084(.a(new_n178), .b(new_n179), .out0(new_n180));
  aoai13aa1n03x5               g085(.a(new_n180), .b(new_n174), .c(new_n171), .d(new_n175), .o1(new_n181));
  oai022aa1n02x5               g086(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n182));
  aoai13aa1n02x5               g087(.a(new_n179), .b(new_n176), .c(new_n173), .d(new_n170), .o1(new_n183));
  oai012aa1n02x5               g088(.a(new_n181), .b(new_n183), .c(new_n182), .o1(\s[16] ));
  nano23aa1n02x4               g089(.a(new_n174), .b(new_n178), .c(new_n179), .d(new_n175), .out0(new_n185));
  nano22aa1n03x7               g090(.a(new_n149), .b(new_n168), .c(new_n185), .out0(new_n186));
  aoai13aa1n09x5               g091(.a(new_n186), .b(new_n146), .c(new_n122), .d(new_n125), .o1(new_n187));
  nano22aa1n02x4               g092(.a(new_n178), .b(new_n175), .c(new_n179), .out0(new_n188));
  oai012aa1n02x5               g093(.a(new_n140), .b(\b[12] ), .c(\a[13] ), .o1(new_n189));
  nona23aa1n03x5               g094(.a(new_n166), .b(new_n157), .c(new_n174), .d(new_n163), .out0(new_n190));
  nor002aa1n02x5               g095(.a(new_n190), .b(new_n189), .o1(new_n191));
  oai122aa1n02x7               g096(.a(new_n166), .b(new_n163), .c(new_n156), .d(\b[14] ), .e(\a[15] ), .o1(new_n192));
  oai012aa1n02x5               g097(.a(new_n179), .b(new_n178), .c(new_n174), .o1(new_n193));
  oaib12aa1n03x5               g098(.a(new_n193), .b(new_n192), .c(new_n188), .out0(new_n194));
  aoi013aa1n06x4               g099(.a(new_n194), .b(new_n191), .c(new_n153), .d(new_n188), .o1(new_n195));
  nanp02aa1n09x5               g100(.a(new_n187), .b(new_n195), .o1(new_n196));
  xorc02aa1n02x5               g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  aoi012aa1n02x5               g102(.a(new_n197), .b(new_n179), .c(new_n182), .o1(new_n198));
  nona22aa1n02x4               g103(.a(new_n171), .b(new_n176), .c(new_n180), .out0(new_n199));
  aoi022aa1n02x5               g104(.a(new_n199), .b(new_n198), .c(new_n196), .d(new_n197), .o1(\s[17] ));
  norp02aa1n02x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nanp02aa1n04x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  obai22aa1n02x7               g107(.a(new_n202), .b(new_n201), .c(\a[17] ), .d(\b[16] ), .out0(new_n203));
  aoi012aa1n02x5               g108(.a(new_n203), .b(new_n196), .c(new_n197), .o1(new_n204));
  inv000aa1d42x5               g109(.a(\a[17] ), .o1(new_n205));
  inv020aa1n04x5               g110(.a(\a[18] ), .o1(new_n206));
  xroi22aa1d06x4               g111(.a(new_n205), .b(\b[16] ), .c(new_n206), .d(\b[17] ), .out0(new_n207));
  inv000aa1n02x5               g112(.a(new_n207), .o1(new_n208));
  oai022aa1d24x5               g113(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n209));
  nanp02aa1n02x5               g114(.a(new_n209), .b(new_n202), .o1(new_n210));
  aoai13aa1n06x5               g115(.a(new_n210), .b(new_n208), .c(new_n187), .d(new_n195), .o1(new_n211));
  aoib12aa1n02x5               g116(.a(new_n204), .b(new_n211), .c(new_n201), .out0(\s[18] ));
  nor002aa1n03x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nand02aa1n06x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  aoi122aa1n02x5               g120(.a(new_n215), .b(new_n202), .c(new_n209), .d(new_n196), .e(new_n207), .o1(new_n216));
  aoi012aa1n02x5               g121(.a(new_n216), .b(new_n211), .c(new_n215), .o1(\s[19] ));
  xnrc02aa1n02x5               g122(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoi012aa1n03x5               g123(.a(new_n213), .b(new_n211), .c(new_n214), .o1(new_n219));
  orn002aa1n24x5               g124(.a(\a[20] ), .b(\b[19] ), .o(new_n220));
  nand02aa1n08x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nand02aa1n02x5               g126(.a(new_n211), .b(new_n215), .o1(new_n222));
  inv000aa1n02x5               g127(.a(new_n221), .o1(new_n223));
  oai022aa1n02x5               g128(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n224));
  nona22aa1n02x5               g129(.a(new_n222), .b(new_n223), .c(new_n224), .out0(new_n225));
  aoai13aa1n02x5               g130(.a(new_n225), .b(new_n219), .c(new_n221), .d(new_n220), .o1(\s[20] ));
  nano23aa1n09x5               g131(.a(new_n213), .b(new_n223), .c(new_n220), .d(new_n214), .out0(new_n227));
  nand02aa1d04x5               g132(.a(new_n207), .b(new_n227), .o1(new_n228));
  aoi012aa1n06x5               g133(.a(new_n228), .b(new_n187), .c(new_n195), .o1(new_n229));
  nanp03aa1n06x5               g134(.a(new_n220), .b(new_n214), .c(new_n221), .o1(new_n230));
  oai112aa1n06x5               g135(.a(new_n209), .b(new_n202), .c(\b[18] ), .d(\a[19] ), .o1(new_n231));
  nanp02aa1n02x5               g136(.a(new_n224), .b(new_n221), .o1(new_n232));
  oai012aa1n06x5               g137(.a(new_n232), .b(new_n231), .c(new_n230), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[21] ), .b(\b[20] ), .out0(new_n234));
  tech160nm_fioai012aa1n04x5   g139(.a(new_n234), .b(new_n229), .c(new_n233), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n234), .b(new_n221), .c(new_n224), .o1(new_n236));
  oai012aa1n02x5               g141(.a(new_n236), .b(new_n231), .c(new_n230), .o1(new_n237));
  oa0012aa1n02x5               g142(.a(new_n235), .b(new_n237), .c(new_n229), .o(\s[21] ));
  inv000aa1d42x5               g143(.a(\a[21] ), .o1(new_n239));
  nanb02aa1n02x5               g144(.a(\b[20] ), .b(new_n239), .out0(new_n240));
  xorc02aa1n02x5               g145(.a(\a[22] ), .b(\b[21] ), .out0(new_n241));
  and002aa1n02x5               g146(.a(\b[21] ), .b(\a[22] ), .o(new_n242));
  oai022aa1n02x5               g147(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n243));
  nona22aa1n02x5               g148(.a(new_n235), .b(new_n242), .c(new_n243), .out0(new_n244));
  aoai13aa1n03x5               g149(.a(new_n244), .b(new_n241), .c(new_n240), .d(new_n235), .o1(\s[22] ));
  inv000aa1d42x5               g150(.a(\a[22] ), .o1(new_n246));
  xroi22aa1d06x4               g151(.a(new_n239), .b(\b[20] ), .c(new_n246), .d(\b[21] ), .out0(new_n247));
  nano22aa1n02x4               g152(.a(new_n208), .b(new_n247), .c(new_n227), .out0(new_n248));
  oaoi03aa1n02x5               g153(.a(\a[22] ), .b(\b[21] ), .c(new_n240), .o1(new_n249));
  tech160nm_fiao0012aa1n02p5x5 g154(.a(new_n249), .b(new_n233), .c(new_n247), .o(new_n250));
  tech160nm_fixorc02aa1n05x5   g155(.a(\a[23] ), .b(\b[22] ), .out0(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n250), .c(new_n196), .d(new_n248), .o1(new_n252));
  aoi112aa1n02x5               g157(.a(new_n251), .b(new_n249), .c(new_n233), .d(new_n247), .o1(new_n253));
  aobi12aa1n02x5               g158(.a(new_n253), .b(new_n196), .c(new_n248), .out0(new_n254));
  norb02aa1n03x4               g159(.a(new_n252), .b(new_n254), .out0(\s[23] ));
  norp02aa1n02x5               g160(.a(\b[22] ), .b(\a[23] ), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  tech160nm_fixorc02aa1n03p5x5 g162(.a(\a[24] ), .b(\b[23] ), .out0(new_n258));
  and002aa1n02x5               g163(.a(\b[23] ), .b(\a[24] ), .o(new_n259));
  oai022aa1n02x5               g164(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n260));
  nona22aa1n02x5               g165(.a(new_n252), .b(new_n259), .c(new_n260), .out0(new_n261));
  aoai13aa1n03x5               g166(.a(new_n261), .b(new_n258), .c(new_n257), .d(new_n252), .o1(\s[24] ));
  nano32aa1n02x5               g167(.a(new_n228), .b(new_n258), .c(new_n247), .d(new_n251), .out0(new_n263));
  and002aa1n02x7               g168(.a(new_n258), .b(new_n251), .o(new_n264));
  aoai13aa1n04x5               g169(.a(new_n264), .b(new_n249), .c(new_n233), .d(new_n247), .o1(new_n265));
  aob012aa1n02x5               g170(.a(new_n260), .b(\b[23] ), .c(\a[24] ), .out0(new_n266));
  nanp02aa1n02x5               g171(.a(new_n265), .b(new_n266), .o1(new_n267));
  xorc02aa1n12x5               g172(.a(\a[25] ), .b(\b[24] ), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n267), .c(new_n196), .d(new_n263), .o1(new_n269));
  nanb02aa1n02x5               g174(.a(new_n268), .b(new_n266), .out0(new_n270));
  aoi122aa1n02x7               g175(.a(new_n270), .b(new_n250), .c(new_n264), .d(new_n196), .e(new_n263), .o1(new_n271));
  norb02aa1n02x7               g176(.a(new_n269), .b(new_n271), .out0(\s[25] ));
  nor042aa1n06x5               g177(.a(\b[24] ), .b(\a[25] ), .o1(new_n273));
  inv040aa1n03x5               g178(.a(new_n273), .o1(new_n274));
  xorc02aa1n02x5               g179(.a(\a[26] ), .b(\b[25] ), .out0(new_n275));
  and002aa1n02x5               g180(.a(\b[25] ), .b(\a[26] ), .o(new_n276));
  oai022aa1n02x5               g181(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n277));
  nona22aa1n02x5               g182(.a(new_n269), .b(new_n276), .c(new_n277), .out0(new_n278));
  aoai13aa1n03x5               g183(.a(new_n278), .b(new_n275), .c(new_n274), .d(new_n269), .o1(\s[26] ));
  nanp02aa1n02x5               g184(.a(new_n275), .b(new_n268), .o1(new_n280));
  nona23aa1n03x5               g185(.a(new_n247), .b(new_n264), .c(new_n228), .d(new_n280), .out0(new_n281));
  aoi012aa1n12x5               g186(.a(new_n281), .b(new_n187), .c(new_n195), .o1(new_n282));
  tech160nm_fioaoi03aa1n02p5x5 g187(.a(\a[26] ), .b(\b[25] ), .c(new_n274), .o1(new_n283));
  inv000aa1n02x5               g188(.a(new_n283), .o1(new_n284));
  aoai13aa1n06x5               g189(.a(new_n284), .b(new_n280), .c(new_n265), .d(new_n266), .o1(new_n285));
  xorc02aa1n02x5               g190(.a(\a[27] ), .b(\b[26] ), .out0(new_n286));
  oa0012aa1n03x5               g191(.a(new_n286), .b(new_n285), .c(new_n282), .o(new_n287));
  aoi113aa1n02x5               g192(.a(new_n283), .b(new_n286), .c(new_n267), .d(new_n268), .e(new_n275), .o1(new_n288));
  aoib12aa1n02x7               g193(.a(new_n287), .b(new_n288), .c(new_n282), .out0(\s[27] ));
  norp02aa1n02x5               g194(.a(\b[26] ), .b(\a[27] ), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n290), .o1(new_n291));
  tech160nm_fioai012aa1n04x5   g196(.a(new_n286), .b(new_n285), .c(new_n282), .o1(new_n292));
  xorc02aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .out0(new_n293));
  oai022aa1d24x5               g198(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n294));
  aoi012aa1n02x5               g199(.a(new_n294), .b(\a[28] ), .c(\b[27] ), .o1(new_n295));
  tech160nm_finand02aa1n03p5x5 g200(.a(new_n292), .b(new_n295), .o1(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n293), .c(new_n292), .d(new_n291), .o1(\s[28] ));
  xorc02aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .out0(new_n298));
  and002aa1n02x5               g203(.a(new_n293), .b(new_n286), .o(new_n299));
  oaih12aa1n02x5               g204(.a(new_n299), .b(new_n285), .c(new_n282), .o1(new_n300));
  inv000aa1d42x5               g205(.a(\b[27] ), .o1(new_n301));
  oaib12aa1n18x5               g206(.a(new_n294), .b(new_n301), .c(\a[28] ), .out0(new_n302));
  nanp03aa1n03x5               g207(.a(new_n300), .b(new_n302), .c(new_n298), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n302), .o1(new_n304));
  oaoi13aa1n03x5               g209(.a(new_n304), .b(new_n299), .c(new_n285), .d(new_n282), .o1(new_n305));
  oaih12aa1n02x5               g210(.a(new_n303), .b(new_n305), .c(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g211(.a(new_n114), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g212(.a(new_n286), .b(new_n298), .c(new_n293), .o(new_n308));
  tech160nm_fioaoi03aa1n03p5x5 g213(.a(\a[29] ), .b(\b[28] ), .c(new_n302), .o1(new_n309));
  oaoi13aa1n03x5               g214(.a(new_n309), .b(new_n308), .c(new_n285), .d(new_n282), .o1(new_n310));
  xorc02aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .out0(new_n311));
  oaih12aa1n02x5               g216(.a(new_n308), .b(new_n285), .c(new_n282), .o1(new_n312));
  norb02aa1n03x5               g217(.a(new_n311), .b(new_n309), .out0(new_n313));
  tech160nm_finand02aa1n03p5x5 g218(.a(new_n312), .b(new_n313), .o1(new_n314));
  oaih12aa1n02x5               g219(.a(new_n314), .b(new_n310), .c(new_n311), .o1(\s[30] ));
  and003aa1n02x5               g220(.a(new_n299), .b(new_n311), .c(new_n298), .o(new_n316));
  aoi012aa1n02x5               g221(.a(new_n313), .b(\a[30] ), .c(\b[29] ), .o1(new_n317));
  oaoi13aa1n03x5               g222(.a(new_n317), .b(new_n316), .c(new_n285), .d(new_n282), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[31] ), .b(\b[30] ), .out0(new_n319));
  oaih12aa1n02x5               g224(.a(new_n316), .b(new_n285), .c(new_n282), .o1(new_n320));
  norb02aa1n02x5               g225(.a(new_n319), .b(new_n317), .out0(new_n321));
  tech160nm_finand02aa1n03p5x5 g226(.a(new_n320), .b(new_n321), .o1(new_n322));
  oaih12aa1n02x5               g227(.a(new_n322), .b(new_n318), .c(new_n319), .o1(\s[31] ));
  nanb02aa1n02x5               g228(.a(new_n118), .b(new_n119), .out0(new_n324));
  nanb03aa1n02x5               g229(.a(new_n112), .b(new_n114), .c(new_n113), .out0(new_n325));
  nona23aa1n02x4               g230(.a(new_n325), .b(new_n119), .c(new_n118), .d(new_n112), .out0(new_n326));
  oaib12aa1n02x5               g231(.a(new_n326), .b(new_n115), .c(new_n324), .out0(\s[3] ));
  nanb02aa1n02x5               g232(.a(new_n116), .b(new_n117), .out0(new_n328));
  nanp02aa1n02x5               g233(.a(new_n326), .b(new_n119), .o1(new_n329));
  aboi22aa1n03x5               g234(.a(new_n116), .b(new_n122), .c(new_n329), .d(new_n328), .out0(\s[4] ));
  oaoi13aa1n04x5               g235(.a(new_n124), .b(new_n121), .c(new_n120), .d(new_n115), .o1(new_n331));
  oai112aa1n02x5               g236(.a(new_n121), .b(new_n124), .c(new_n120), .d(new_n115), .o1(new_n332));
  norb02aa1n02x5               g237(.a(new_n332), .b(new_n331), .out0(\s[5] ));
  oabi12aa1n02x5               g238(.a(new_n123), .b(new_n331), .c(new_n109), .out0(new_n334));
  norb03aa1n02x5               g239(.a(new_n123), .b(new_n331), .c(new_n109), .out0(new_n335));
  norb02aa1n02x5               g240(.a(new_n334), .b(new_n335), .out0(\s[6] ));
  nanb02aa1n02x5               g241(.a(new_n104), .b(new_n105), .out0(new_n337));
  nanp02aa1n02x5               g242(.a(new_n108), .b(new_n107), .o1(new_n338));
  xobna2aa1n03x5               g243(.a(new_n337), .b(new_n334), .c(new_n338), .out0(\s[7] ));
  norb03aa1n02x5               g244(.a(new_n102), .b(new_n104), .c(new_n103), .out0(new_n340));
  tech160nm_fiaoi012aa1n02p5x5 g245(.a(new_n337), .b(new_n334), .c(new_n338), .o1(new_n341));
  obai22aa1n02x7               g246(.a(new_n102), .b(new_n103), .c(new_n341), .d(new_n104), .out0(new_n342));
  oaib12aa1n02x5               g247(.a(new_n342), .b(new_n341), .c(new_n340), .out0(\s[8] ));
  aoi012aa1n02x5               g248(.a(new_n127), .b(new_n122), .c(new_n125), .o1(new_n344));
  norb02aa1n02x5               g249(.a(new_n100), .b(new_n101), .out0(new_n345));
  oab012aa1n02x4               g250(.a(new_n345), .b(new_n106), .c(new_n110), .out0(new_n346));
  aoi022aa1n02x5               g251(.a(new_n161), .b(new_n345), .c(new_n344), .d(new_n346), .o1(\s[9] ));
endmodule


