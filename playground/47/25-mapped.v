// Benchmark "adder" written by ABC on Thu Jul 18 12:18:07 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n334, new_n336, new_n338, new_n340,
    new_n341, new_n342, new_n343, new_n344, new_n345, new_n347, new_n348,
    new_n350, new_n352, new_n353;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_fixorc02aa1n03p5x5 g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  orn002aa1n02x5               g002(.a(\a[9] ), .b(\b[8] ), .o(new_n98));
  orn002aa1n24x5               g003(.a(\a[2] ), .b(\b[1] ), .o(new_n99));
  nand02aa1d16x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n12x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  aob012aa1d18x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .out0(new_n102));
  nor042aa1n06x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand22aa1n09x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norb02aa1n12x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  nor042aa1n09x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand22aa1n06x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  norb02aa1n15x5               g012(.a(new_n107), .b(new_n106), .out0(new_n108));
  nand23aa1d12x5               g013(.a(new_n102), .b(new_n105), .c(new_n108), .o1(new_n109));
  oaih12aa1n06x5               g014(.a(new_n104), .b(new_n106), .c(new_n103), .o1(new_n110));
  nand02aa1d24x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  oai012aa1n02x5               g016(.a(new_n111), .b(\b[6] ), .c(\a[7] ), .o1(new_n112));
  inv040aa1d32x5               g017(.a(\a[8] ), .o1(new_n113));
  inv040aa1d28x5               g018(.a(\b[7] ), .o1(new_n114));
  aoi022aa1n06x5               g019(.a(new_n114), .b(new_n113), .c(\a[7] ), .d(\b[6] ), .o1(new_n115));
  oai022aa1d18x5               g020(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n116));
  aoi022aa1n06x5               g021(.a(\b[5] ), .b(\a[6] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n117));
  nona23aa1n12x5               g022(.a(new_n117), .b(new_n115), .c(new_n112), .d(new_n116), .out0(new_n118));
  nor042aa1d18x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nand02aa1d08x5               g024(.a(new_n114), .b(new_n113), .o1(new_n120));
  aob012aa1n06x5               g025(.a(new_n120), .b(new_n119), .c(new_n111), .out0(new_n121));
  nand22aa1n04x5               g026(.a(\b[6] ), .b(\a[7] ), .o1(new_n122));
  inv020aa1n04x5               g027(.a(new_n119), .o1(new_n123));
  nand02aa1n04x5               g028(.a(new_n116), .b(new_n123), .o1(new_n124));
  aoi022aa1d24x5               g029(.a(\b[7] ), .b(\a[8] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n125));
  nano32aa1n03x7               g030(.a(new_n124), .b(new_n125), .c(new_n120), .d(new_n122), .out0(new_n126));
  nor042aa1n03x5               g031(.a(new_n126), .b(new_n121), .o1(new_n127));
  aoai13aa1n12x5               g032(.a(new_n127), .b(new_n118), .c(new_n109), .d(new_n110), .o1(new_n128));
  tech160nm_fixorc02aa1n03p5x5 g033(.a(\a[9] ), .b(\b[8] ), .out0(new_n129));
  nand22aa1n03x5               g034(.a(new_n128), .b(new_n129), .o1(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n97), .b(new_n130), .c(new_n98), .out0(\s[10] ));
  oa0022aa1n06x5               g036(.a(\b[9] ), .b(\a[10] ), .c(\b[8] ), .d(\a[9] ), .o(new_n132));
  nand22aa1n03x5               g037(.a(new_n130), .b(new_n132), .o1(new_n133));
  nor002aa1n16x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  aoi022aa1d24x5               g039(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  nand42aa1n06x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nanb02aa1n02x5               g042(.a(new_n134), .b(new_n137), .out0(new_n138));
  aob012aa1n02x5               g043(.a(new_n133), .b(\b[9] ), .c(\a[10] ), .out0(new_n139));
  aoi022aa1n02x5               g044(.a(new_n139), .b(new_n138), .c(new_n133), .d(new_n136), .o1(\s[11] ));
  tech160nm_fiaoi012aa1n05x5   g045(.a(new_n134), .b(new_n133), .c(new_n135), .o1(new_n141));
  xnrb03aa1n03x5               g046(.a(new_n141), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor042aa1n06x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanp02aa1n12x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nano23aa1n03x7               g049(.a(new_n134), .b(new_n143), .c(new_n144), .d(new_n137), .out0(new_n145));
  and003aa1n02x5               g050(.a(new_n145), .b(new_n129), .c(new_n97), .o(new_n146));
  nanp02aa1n06x5               g051(.a(new_n128), .b(new_n146), .o1(new_n147));
  norb03aa1n03x5               g052(.a(new_n144), .b(new_n134), .c(new_n143), .out0(new_n148));
  nanb03aa1n09x5               g053(.a(new_n132), .b(new_n148), .c(new_n135), .out0(new_n149));
  aoi012aa1d18x5               g054(.a(new_n143), .b(new_n134), .c(new_n144), .o1(new_n150));
  nanp02aa1n06x5               g055(.a(new_n149), .b(new_n150), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  nanp02aa1n06x5               g057(.a(new_n147), .b(new_n152), .o1(new_n153));
  nor042aa1n04x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nand42aa1d28x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  nano22aa1n02x4               g061(.a(new_n156), .b(new_n149), .c(new_n150), .out0(new_n157));
  aoi022aa1n02x5               g062(.a(new_n153), .b(new_n156), .c(new_n147), .d(new_n157), .o1(\s[13] ));
  tech160nm_fiaoi012aa1n05x5   g063(.a(new_n154), .b(new_n153), .c(new_n155), .o1(new_n159));
  xnrb03aa1n03x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n03x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand42aa1n16x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nano23aa1d15x5               g067(.a(new_n154), .b(new_n161), .c(new_n162), .d(new_n155), .out0(new_n163));
  aoai13aa1n06x5               g068(.a(new_n163), .b(new_n151), .c(new_n128), .d(new_n146), .o1(new_n164));
  oa0012aa1n02x5               g069(.a(new_n162), .b(new_n161), .c(new_n154), .o(new_n165));
  nanb02aa1n06x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  norp02aa1n09x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nand02aa1n04x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  norb02aa1n06x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  oaih22aa1d12x5               g074(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n170));
  aboi22aa1n03x5               g075(.a(new_n167), .b(new_n168), .c(new_n170), .d(new_n162), .out0(new_n171));
  aoi022aa1n02x5               g076(.a(new_n166), .b(new_n169), .c(new_n164), .d(new_n171), .o1(\s[15] ));
  xorc02aa1n12x5               g077(.a(\a[16] ), .b(\b[15] ), .out0(new_n173));
  inv000aa1d42x5               g078(.a(new_n173), .o1(new_n174));
  aoai13aa1n03x5               g079(.a(new_n174), .b(new_n167), .c(new_n166), .d(new_n168), .o1(new_n175));
  aoai13aa1n03x5               g080(.a(new_n169), .b(new_n165), .c(new_n153), .d(new_n163), .o1(new_n176));
  nona22aa1n03x5               g081(.a(new_n176), .b(new_n174), .c(new_n167), .out0(new_n177));
  nanp02aa1n03x5               g082(.a(new_n175), .b(new_n177), .o1(\s[16] ));
  nand03aa1d16x5               g083(.a(new_n163), .b(new_n169), .c(new_n173), .o1(new_n179));
  nano32aa1d12x5               g084(.a(new_n179), .b(new_n145), .c(new_n129), .d(new_n97), .out0(new_n180));
  nanp02aa1n06x5               g085(.a(new_n128), .b(new_n180), .o1(new_n181));
  inv000aa1d42x5               g086(.a(\a[16] ), .o1(new_n182));
  inv000aa1d42x5               g087(.a(\b[15] ), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(new_n183), .b(new_n182), .o1(new_n184));
  nanp02aa1n02x5               g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  nanp03aa1n02x5               g090(.a(new_n184), .b(new_n168), .c(new_n185), .o1(new_n186));
  oai112aa1n02x7               g091(.a(new_n170), .b(new_n162), .c(\b[14] ), .d(\a[15] ), .o1(new_n187));
  tech160nm_fioaoi03aa1n03p5x5 g092(.a(new_n182), .b(new_n183), .c(new_n167), .o1(new_n188));
  oa0012aa1n06x5               g093(.a(new_n188), .b(new_n187), .c(new_n186), .o(new_n189));
  aoai13aa1n12x5               g094(.a(new_n189), .b(new_n179), .c(new_n149), .d(new_n150), .o1(new_n190));
  inv030aa1n03x5               g095(.a(new_n190), .o1(new_n191));
  nand02aa1d08x5               g096(.a(new_n191), .b(new_n181), .o1(new_n192));
  xorc02aa1n02x5               g097(.a(\a[17] ), .b(\b[16] ), .out0(new_n193));
  xnrc02aa1n12x5               g098(.a(\b[16] ), .b(\a[17] ), .out0(new_n194));
  oai112aa1n02x5               g099(.a(new_n188), .b(new_n194), .c(new_n187), .d(new_n186), .o1(new_n195));
  aoib12aa1n02x5               g100(.a(new_n195), .b(new_n151), .c(new_n179), .out0(new_n196));
  aoi022aa1n02x5               g101(.a(new_n192), .b(new_n193), .c(new_n181), .d(new_n196), .o1(\s[17] ));
  inv040aa1d32x5               g102(.a(\a[18] ), .o1(new_n198));
  nor022aa1n08x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  tech160nm_fiaoi012aa1n05x5   g104(.a(new_n199), .b(new_n192), .c(new_n193), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[17] ), .c(new_n198), .out0(\s[18] ));
  inv000aa1d42x5               g106(.a(\b[17] ), .o1(new_n202));
  nand02aa1n03x5               g107(.a(new_n202), .b(new_n198), .o1(new_n203));
  nand42aa1n08x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nano22aa1d15x5               g109(.a(new_n194), .b(new_n203), .c(new_n204), .out0(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n190), .c(new_n128), .d(new_n180), .o1(new_n206));
  tech160nm_fiaoi012aa1n04x5   g111(.a(new_n199), .b(new_n198), .c(new_n202), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n204), .b(new_n207), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  nor042aa1n04x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nand42aa1n06x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  norb02aa1n12x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n206), .c(new_n209), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand02aa1d06x5               g119(.a(new_n206), .b(new_n209), .o1(new_n215));
  nor042aa1d18x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nand42aa1d28x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nanb02aa1n12x5               g122(.a(new_n216), .b(new_n217), .out0(new_n218));
  aoai13aa1n03x5               g123(.a(new_n218), .b(new_n210), .c(new_n215), .d(new_n211), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(new_n215), .b(new_n212), .o1(new_n220));
  nona22aa1n02x4               g125(.a(new_n220), .b(new_n218), .c(new_n210), .out0(new_n221));
  nanp02aa1n03x5               g126(.a(new_n221), .b(new_n219), .o1(\s[20] ));
  nanb03aa1d24x5               g127(.a(new_n218), .b(new_n205), .c(new_n212), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n190), .c(new_n128), .d(new_n180), .o1(new_n225));
  nanb03aa1n02x5               g130(.a(new_n216), .b(new_n217), .c(new_n211), .out0(new_n226));
  oai022aa1n02x5               g131(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n227));
  oai112aa1n02x5               g132(.a(new_n227), .b(new_n204), .c(\b[18] ), .d(\a[19] ), .o1(new_n228));
  aoi012aa1n09x5               g133(.a(new_n216), .b(new_n210), .c(new_n217), .o1(new_n229));
  oai012aa1n06x5               g134(.a(new_n229), .b(new_n228), .c(new_n226), .o1(new_n230));
  nanb02aa1n06x5               g135(.a(new_n230), .b(new_n225), .out0(new_n231));
  xorc02aa1n12x5               g136(.a(\a[21] ), .b(\b[20] ), .out0(new_n232));
  nano22aa1d15x5               g137(.a(new_n216), .b(new_n211), .c(new_n217), .out0(new_n233));
  tech160nm_fioai012aa1n05x5   g138(.a(new_n204), .b(\b[18] ), .c(\a[19] ), .o1(new_n234));
  norp02aa1n02x5               g139(.a(new_n207), .b(new_n234), .o1(new_n235));
  inv040aa1n03x5               g140(.a(new_n229), .o1(new_n236));
  aoi112aa1n02x5               g141(.a(new_n236), .b(new_n232), .c(new_n235), .d(new_n233), .o1(new_n237));
  aoi022aa1n02x5               g142(.a(new_n231), .b(new_n232), .c(new_n225), .d(new_n237), .o1(\s[21] ));
  nor042aa1n09x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  xnrc02aa1n12x5               g144(.a(\b[21] ), .b(\a[22] ), .out0(new_n240));
  aoai13aa1n03x5               g145(.a(new_n240), .b(new_n239), .c(new_n231), .d(new_n232), .o1(new_n241));
  aoai13aa1n03x5               g146(.a(new_n232), .b(new_n230), .c(new_n192), .d(new_n224), .o1(new_n242));
  nona22aa1n03x5               g147(.a(new_n242), .b(new_n240), .c(new_n239), .out0(new_n243));
  nanp02aa1n03x5               g148(.a(new_n241), .b(new_n243), .o1(\s[22] ));
  xnrc02aa1n02x5               g149(.a(\b[20] ), .b(\a[21] ), .out0(new_n245));
  nor042aa1n02x5               g150(.a(new_n240), .b(new_n245), .o1(new_n246));
  nano32aa1n02x4               g151(.a(new_n218), .b(new_n246), .c(new_n205), .d(new_n212), .out0(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n190), .c(new_n128), .d(new_n180), .o1(new_n248));
  nona22aa1n09x5               g153(.a(new_n233), .b(new_n207), .c(new_n234), .out0(new_n249));
  nanb02aa1n03x5               g154(.a(new_n240), .b(new_n232), .out0(new_n250));
  inv000aa1d42x5               g155(.a(\a[22] ), .o1(new_n251));
  inv000aa1d42x5               g156(.a(\b[21] ), .o1(new_n252));
  oao003aa1n02x5               g157(.a(new_n251), .b(new_n252), .c(new_n239), .carry(new_n253));
  inv040aa1n06x5               g158(.a(new_n253), .o1(new_n254));
  aoai13aa1n12x5               g159(.a(new_n254), .b(new_n250), .c(new_n249), .d(new_n229), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  nanp02aa1n06x5               g161(.a(new_n248), .b(new_n256), .o1(new_n257));
  xorc02aa1n12x5               g162(.a(\a[23] ), .b(\b[22] ), .out0(new_n258));
  aoi112aa1n02x5               g163(.a(new_n258), .b(new_n253), .c(new_n230), .d(new_n246), .o1(new_n259));
  aoi022aa1n02x5               g164(.a(new_n257), .b(new_n258), .c(new_n248), .d(new_n259), .o1(\s[23] ));
  norp02aa1n02x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  xnrc02aa1n12x5               g166(.a(\b[23] ), .b(\a[24] ), .out0(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n261), .c(new_n257), .d(new_n258), .o1(new_n263));
  nanp02aa1n02x5               g168(.a(new_n257), .b(new_n258), .o1(new_n264));
  nona22aa1n03x5               g169(.a(new_n264), .b(new_n262), .c(new_n261), .out0(new_n265));
  nanp02aa1n03x5               g170(.a(new_n265), .b(new_n263), .o1(\s[24] ));
  norb02aa1n03x5               g171(.a(new_n258), .b(new_n262), .out0(new_n267));
  nano22aa1n06x5               g172(.a(new_n223), .b(new_n246), .c(new_n267), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n190), .c(new_n128), .d(new_n180), .o1(new_n269));
  aoai13aa1n03x5               g174(.a(new_n246), .b(new_n236), .c(new_n235), .d(new_n233), .o1(new_n270));
  inv000aa1n02x5               g175(.a(new_n267), .o1(new_n271));
  inv000aa1d42x5               g176(.a(\a[24] ), .o1(new_n272));
  inv000aa1d42x5               g177(.a(\b[23] ), .o1(new_n273));
  oao003aa1n02x5               g178(.a(new_n272), .b(new_n273), .c(new_n261), .carry(new_n274));
  inv000aa1n02x5               g179(.a(new_n274), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n271), .c(new_n270), .d(new_n254), .o1(new_n276));
  nanb02aa1n06x5               g181(.a(new_n276), .b(new_n269), .out0(new_n277));
  xorc02aa1n12x5               g182(.a(\a[25] ), .b(\b[24] ), .out0(new_n278));
  aoi112aa1n02x5               g183(.a(new_n278), .b(new_n274), .c(new_n255), .d(new_n267), .o1(new_n279));
  aoi022aa1n02x5               g184(.a(new_n277), .b(new_n278), .c(new_n269), .d(new_n279), .o1(\s[25] ));
  norp02aa1n02x5               g185(.a(\b[24] ), .b(\a[25] ), .o1(new_n281));
  nor022aa1n08x5               g186(.a(\b[25] ), .b(\a[26] ), .o1(new_n282));
  nand42aa1n03x5               g187(.a(\b[25] ), .b(\a[26] ), .o1(new_n283));
  nanb02aa1n06x5               g188(.a(new_n282), .b(new_n283), .out0(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n281), .c(new_n277), .d(new_n278), .o1(new_n285));
  aoai13aa1n03x5               g190(.a(new_n278), .b(new_n276), .c(new_n192), .d(new_n268), .o1(new_n286));
  nona22aa1n03x5               g191(.a(new_n286), .b(new_n284), .c(new_n281), .out0(new_n287));
  nanp02aa1n03x5               g192(.a(new_n285), .b(new_n287), .o1(\s[26] ));
  norb02aa1n06x5               g193(.a(new_n278), .b(new_n284), .out0(new_n289));
  nano32aa1n03x7               g194(.a(new_n223), .b(new_n289), .c(new_n246), .d(new_n267), .out0(new_n290));
  aoai13aa1n06x5               g195(.a(new_n290), .b(new_n190), .c(new_n128), .d(new_n180), .o1(new_n291));
  oai012aa1n02x5               g196(.a(new_n283), .b(new_n282), .c(new_n281), .o1(new_n292));
  aobi12aa1n06x5               g197(.a(new_n292), .b(new_n276), .c(new_n289), .out0(new_n293));
  xorc02aa1n12x5               g198(.a(\a[27] ), .b(\b[26] ), .out0(new_n294));
  xnbna2aa1n03x5               g199(.a(new_n294), .b(new_n293), .c(new_n291), .out0(\s[27] ));
  aoai13aa1n04x5               g200(.a(new_n289), .b(new_n274), .c(new_n255), .d(new_n267), .o1(new_n296));
  nand23aa1n06x5               g201(.a(new_n291), .b(new_n296), .c(new_n292), .o1(new_n297));
  norp02aa1n02x5               g202(.a(\b[26] ), .b(\a[27] ), .o1(new_n298));
  norp02aa1n02x5               g203(.a(\b[27] ), .b(\a[28] ), .o1(new_n299));
  nanp02aa1n02x5               g204(.a(\b[27] ), .b(\a[28] ), .o1(new_n300));
  nanb02aa1n02x5               g205(.a(new_n299), .b(new_n300), .out0(new_n301));
  aoai13aa1n03x5               g206(.a(new_n301), .b(new_n298), .c(new_n297), .d(new_n294), .o1(new_n302));
  aoai13aa1n02x5               g207(.a(new_n267), .b(new_n253), .c(new_n230), .d(new_n246), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n289), .o1(new_n304));
  aoai13aa1n03x5               g209(.a(new_n292), .b(new_n304), .c(new_n303), .d(new_n275), .o1(new_n305));
  aoai13aa1n03x5               g210(.a(new_n294), .b(new_n305), .c(new_n192), .d(new_n290), .o1(new_n306));
  nona22aa1n02x5               g211(.a(new_n306), .b(new_n301), .c(new_n298), .out0(new_n307));
  nanp02aa1n03x5               g212(.a(new_n302), .b(new_n307), .o1(\s[28] ));
  norb02aa1n02x5               g213(.a(new_n294), .b(new_n301), .out0(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n305), .c(new_n192), .d(new_n290), .o1(new_n310));
  inv000aa1n02x5               g215(.a(new_n309), .o1(new_n311));
  aoi012aa1n02x5               g216(.a(new_n299), .b(new_n298), .c(new_n300), .o1(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n311), .c(new_n293), .d(new_n291), .o1(new_n313));
  tech160nm_fixorc02aa1n03p5x5 g218(.a(\a[29] ), .b(\b[28] ), .out0(new_n314));
  norb02aa1n02x5               g219(.a(new_n312), .b(new_n314), .out0(new_n315));
  aoi022aa1n03x5               g220(.a(new_n313), .b(new_n314), .c(new_n310), .d(new_n315), .o1(\s[29] ));
  xorb03aa1n02x5               g221(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanb03aa1n02x5               g222(.a(new_n301), .b(new_n314), .c(new_n294), .out0(new_n318));
  nanb02aa1n03x5               g223(.a(new_n318), .b(new_n297), .out0(new_n319));
  oao003aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .c(new_n312), .carry(new_n320));
  aoai13aa1n02x7               g225(.a(new_n320), .b(new_n318), .c(new_n293), .d(new_n291), .o1(new_n321));
  xorc02aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .out0(new_n322));
  norb02aa1n02x5               g227(.a(new_n320), .b(new_n322), .out0(new_n323));
  aoi022aa1n03x5               g228(.a(new_n321), .b(new_n322), .c(new_n319), .d(new_n323), .o1(\s[30] ));
  inv000aa1d42x5               g229(.a(new_n314), .o1(new_n325));
  inv000aa1n02x5               g230(.a(new_n322), .o1(new_n326));
  nona32aa1n03x5               g231(.a(new_n297), .b(new_n326), .c(new_n325), .d(new_n311), .out0(new_n327));
  xorc02aa1n02x5               g232(.a(\a[31] ), .b(\b[30] ), .out0(new_n328));
  oao003aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .c(new_n320), .carry(new_n329));
  norb02aa1n02x5               g234(.a(new_n329), .b(new_n328), .out0(new_n330));
  nona22aa1n02x4               g235(.a(new_n309), .b(new_n325), .c(new_n326), .out0(new_n331));
  aoai13aa1n03x5               g236(.a(new_n329), .b(new_n331), .c(new_n293), .d(new_n291), .o1(new_n332));
  aoi022aa1n03x5               g237(.a(new_n332), .b(new_n328), .c(new_n327), .d(new_n330), .o1(\s[31] ));
  nanp03aa1n02x5               g238(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n334));
  xnbna2aa1n03x5               g239(.a(new_n108), .b(new_n334), .c(new_n99), .out0(\s[3] ));
  aoi012aa1n02x5               g240(.a(new_n106), .b(new_n102), .c(new_n107), .o1(new_n336));
  xnrc02aa1n02x5               g241(.a(new_n336), .b(new_n105), .out0(\s[4] ));
  xorc02aa1n02x5               g242(.a(\a[5] ), .b(\b[4] ), .out0(new_n338));
  xnbna2aa1n03x5               g243(.a(new_n338), .b(new_n109), .c(new_n110), .out0(\s[5] ));
  nanp02aa1n03x5               g244(.a(new_n109), .b(new_n110), .o1(new_n340));
  nanp02aa1n03x5               g245(.a(new_n340), .b(new_n338), .o1(new_n341));
  xorc02aa1n02x5               g246(.a(\a[6] ), .b(\b[5] ), .out0(new_n342));
  oaoi13aa1n02x5               g247(.a(new_n342), .b(new_n341), .c(\a[5] ), .d(\b[4] ), .o1(new_n343));
  and002aa1n02x5               g248(.a(\b[5] ), .b(\a[6] ), .o(new_n344));
  nona22aa1n03x5               g249(.a(new_n341), .b(new_n344), .c(new_n116), .out0(new_n345));
  nanb02aa1n02x5               g250(.a(new_n343), .b(new_n345), .out0(\s[6] ));
  aboi22aa1n03x5               g251(.a(new_n344), .b(new_n345), .c(new_n123), .d(new_n122), .out0(new_n347));
  nona23aa1n03x5               g252(.a(new_n345), .b(new_n122), .c(new_n119), .d(new_n344), .out0(new_n348));
  norb02aa1n02x5               g253(.a(new_n348), .b(new_n347), .out0(\s[7] ));
  xorc02aa1n02x5               g254(.a(\a[8] ), .b(\b[7] ), .out0(new_n350));
  xnbna2aa1n03x5               g255(.a(new_n350), .b(new_n348), .c(new_n123), .out0(\s[8] ));
  aoi012aa1n02x5               g256(.a(new_n118), .b(new_n109), .c(new_n110), .o1(new_n352));
  norp03aa1n02x5               g257(.a(new_n126), .b(new_n129), .c(new_n121), .o1(new_n353));
  aboi22aa1n03x5               g258(.a(new_n352), .b(new_n353), .c(new_n128), .d(new_n129), .out0(\s[9] ));
endmodule

