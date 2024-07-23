// Benchmark "adder" written by ABC on Wed Jul 17 18:23:04 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n297, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n317, new_n319, new_n320, new_n323, new_n325, new_n327;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor042aa1n06x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nand02aa1d24x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nor002aa1d24x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nand02aa1d16x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nano23aa1d15x5               g006(.a(new_n98), .b(new_n100), .c(new_n101), .d(new_n99), .out0(new_n102));
  inv020aa1n02x5               g007(.a(new_n102), .o1(new_n103));
  nor042aa1n06x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand42aa1d28x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  norb02aa1n15x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nor042aa1n04x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  aoi022aa1d24x5               g012(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n108));
  oai022aa1d18x5               g013(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n109));
  oaoi13aa1n09x5               g014(.a(new_n109), .b(new_n106), .c(new_n108), .d(new_n107), .o1(new_n110));
  orn002aa1n24x5               g015(.a(\a[5] ), .b(\b[4] ), .o(new_n111));
  tech160nm_finand02aa1n05x5   g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nand22aa1n04x5               g017(.a(new_n111), .b(new_n112), .o1(new_n113));
  nand42aa1n04x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  xorc02aa1n06x5               g019(.a(\a[6] ), .b(\b[5] ), .out0(new_n115));
  nanb03aa1n12x5               g020(.a(new_n113), .b(new_n115), .c(new_n114), .out0(new_n116));
  oaoi03aa1n09x5               g021(.a(\a[6] ), .b(\b[5] ), .c(new_n111), .o1(new_n117));
  aoi012aa1n02x7               g022(.a(new_n98), .b(new_n100), .c(new_n99), .o1(new_n118));
  aobi12aa1n02x5               g023(.a(new_n118), .b(new_n102), .c(new_n117), .out0(new_n119));
  oai013aa1n09x5               g024(.a(new_n119), .b(new_n116), .c(new_n110), .d(new_n103), .o1(new_n120));
  xorc02aa1n12x5               g025(.a(\a[9] ), .b(\b[8] ), .out0(new_n121));
  nor002aa1d32x5               g026(.a(\b[9] ), .b(\a[10] ), .o1(new_n122));
  nand42aa1d28x5               g027(.a(\b[9] ), .b(\a[10] ), .o1(new_n123));
  norb02aa1n03x5               g028(.a(new_n123), .b(new_n122), .out0(new_n124));
  aoai13aa1n06x5               g029(.a(new_n124), .b(new_n97), .c(new_n120), .d(new_n121), .o1(new_n125));
  aoi112aa1n02x5               g030(.a(new_n124), .b(new_n97), .c(new_n120), .d(new_n121), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n125), .b(new_n126), .out0(\s[10] ));
  oa0012aa1n02x5               g032(.a(new_n123), .b(new_n122), .c(new_n97), .o(new_n128));
  inv000aa1d42x5               g033(.a(new_n128), .o1(new_n129));
  tech160nm_fixorc02aa1n05x5   g034(.a(\a[11] ), .b(\b[10] ), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n125), .c(new_n129), .out0(\s[11] ));
  nand42aa1n04x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  xnrc02aa1n03x5               g037(.a(\b[10] ), .b(\a[11] ), .out0(new_n133));
  nona22aa1n03x5               g038(.a(new_n125), .b(new_n128), .c(new_n133), .out0(new_n134));
  xorc02aa1n12x5               g039(.a(\a[12] ), .b(\b[11] ), .out0(new_n135));
  xobna2aa1n03x5               g040(.a(new_n135), .b(new_n134), .c(new_n132), .out0(\s[12] ));
  oai012aa1n09x5               g041(.a(new_n106), .b(new_n108), .c(new_n107), .o1(new_n137));
  inv000aa1n02x5               g042(.a(new_n109), .o1(new_n138));
  nand02aa1n10x5               g043(.a(new_n137), .b(new_n138), .o1(new_n139));
  xnrc02aa1n02x5               g044(.a(\b[5] ), .b(\a[6] ), .out0(new_n140));
  nano32aa1n03x7               g045(.a(new_n140), .b(new_n111), .c(new_n112), .d(new_n114), .out0(new_n141));
  nand22aa1n03x5               g046(.a(new_n102), .b(new_n117), .o1(new_n142));
  nanp02aa1n06x5               g047(.a(new_n142), .b(new_n118), .o1(new_n143));
  aoi013aa1n09x5               g048(.a(new_n143), .b(new_n139), .c(new_n141), .d(new_n102), .o1(new_n144));
  nanb02aa1n03x5               g049(.a(new_n122), .b(new_n123), .out0(new_n145));
  nona23aa1n02x4               g050(.a(new_n121), .b(new_n135), .c(new_n133), .d(new_n145), .out0(new_n146));
  oai112aa1n06x5               g051(.a(new_n123), .b(new_n132), .c(new_n122), .d(new_n97), .o1(new_n147));
  oa0022aa1n06x5               g052(.a(\a[12] ), .b(\b[11] ), .c(\a[11] ), .d(\b[10] ), .o(new_n148));
  aoi022aa1n12x5               g053(.a(new_n147), .b(new_n148), .c(\b[11] ), .d(\a[12] ), .o1(new_n149));
  inv000aa1n02x5               g054(.a(new_n149), .o1(new_n150));
  tech160nm_fioai012aa1n05x5   g055(.a(new_n150), .b(new_n144), .c(new_n146), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g057(.a(\a[14] ), .o1(new_n153));
  norp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  tech160nm_fixorc02aa1n05x5   g059(.a(\a[13] ), .b(\b[12] ), .out0(new_n155));
  aoi012aa1n02x5               g060(.a(new_n154), .b(new_n151), .c(new_n155), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(new_n153), .out0(\s[14] ));
  inv000aa1d42x5               g062(.a(\a[13] ), .o1(new_n158));
  xroi22aa1d04x5               g063(.a(new_n158), .b(\b[12] ), .c(new_n153), .d(\b[13] ), .out0(new_n159));
  orn002aa1n24x5               g064(.a(\a[13] ), .b(\b[12] ), .o(new_n160));
  oaoi03aa1n12x5               g065(.a(\a[14] ), .b(\b[13] ), .c(new_n160), .o1(new_n161));
  xorc02aa1n12x5               g066(.a(\a[15] ), .b(\b[14] ), .out0(new_n162));
  aoai13aa1n06x5               g067(.a(new_n162), .b(new_n161), .c(new_n151), .d(new_n159), .o1(new_n163));
  aoi112aa1n02x5               g068(.a(new_n162), .b(new_n161), .c(new_n151), .d(new_n159), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(\s[15] ));
  nor002aa1n06x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  xnrc02aa1n12x5               g072(.a(\b[15] ), .b(\a[16] ), .out0(new_n168));
  nanp03aa1n03x5               g073(.a(new_n163), .b(new_n167), .c(new_n168), .o1(new_n169));
  tech160nm_fiaoi012aa1n03p5x5 g074(.a(new_n168), .b(new_n163), .c(new_n167), .o1(new_n170));
  norb02aa1n03x4               g075(.a(new_n169), .b(new_n170), .out0(\s[16] ));
  nand02aa1n02x5               g076(.a(new_n121), .b(new_n124), .o1(new_n172));
  nano22aa1n02x4               g077(.a(new_n172), .b(new_n130), .c(new_n135), .out0(new_n173));
  tech160nm_fixorc02aa1n05x5   g078(.a(\a[14] ), .b(\b[13] ), .out0(new_n174));
  nano32aa1n03x7               g079(.a(new_n168), .b(new_n162), .c(new_n174), .d(new_n155), .out0(new_n175));
  nand22aa1n03x5               g080(.a(new_n175), .b(new_n173), .o1(new_n176));
  aoi022aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .c(\a[15] ), .d(\b[14] ), .o1(new_n177));
  tech160nm_fioai012aa1n05x5   g082(.a(new_n177), .b(new_n161), .c(new_n166), .o1(new_n178));
  tech160nm_fioai012aa1n04x5   g083(.a(new_n178), .b(\b[15] ), .c(\a[16] ), .o1(new_n179));
  aoi012aa1n09x5               g084(.a(new_n179), .b(new_n175), .c(new_n149), .o1(new_n180));
  oai012aa1n18x5               g085(.a(new_n180), .b(new_n144), .c(new_n176), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g087(.a(\a[17] ), .o1(new_n183));
  inv000aa1d42x5               g088(.a(\b[16] ), .o1(new_n184));
  oaoi03aa1n03x5               g089(.a(new_n183), .b(new_n184), .c(new_n181), .o1(new_n185));
  xnrb03aa1n03x5               g090(.a(new_n185), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  xnrc02aa1n02x5               g091(.a(\b[12] ), .b(\a[13] ), .out0(new_n187));
  nona23aa1n02x4               g092(.a(new_n162), .b(new_n174), .c(new_n168), .d(new_n187), .out0(new_n188));
  nor042aa1n02x5               g093(.a(new_n188), .b(new_n146), .o1(new_n189));
  norp02aa1n02x5               g094(.a(\b[15] ), .b(\a[16] ), .o1(new_n190));
  oaoi13aa1n02x5               g095(.a(new_n190), .b(new_n177), .c(new_n161), .d(new_n166), .o1(new_n191));
  tech160nm_fioai012aa1n02p5x5 g096(.a(new_n191), .b(new_n150), .c(new_n188), .o1(new_n192));
  norp02aa1n02x5               g097(.a(\b[16] ), .b(\a[17] ), .o1(new_n193));
  nand42aa1n06x5               g098(.a(\b[16] ), .b(\a[17] ), .o1(new_n194));
  nor002aa1d32x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  nand42aa1n20x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  nano23aa1n06x5               g101(.a(new_n193), .b(new_n195), .c(new_n196), .d(new_n194), .out0(new_n197));
  aoai13aa1n02x5               g102(.a(new_n197), .b(new_n192), .c(new_n120), .d(new_n189), .o1(new_n198));
  aoai13aa1n12x5               g103(.a(new_n196), .b(new_n195), .c(new_n183), .d(new_n184), .o1(new_n199));
  nor002aa1d32x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nand22aa1n04x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  xnbna2aa1n03x5               g107(.a(new_n202), .b(new_n198), .c(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g109(.a(new_n200), .o1(new_n205));
  inv040aa1n02x5               g110(.a(new_n199), .o1(new_n206));
  aoai13aa1n06x5               g111(.a(new_n202), .b(new_n206), .c(new_n181), .d(new_n197), .o1(new_n207));
  nor022aa1n16x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nand22aa1n12x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nanb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  nanp03aa1n03x5               g115(.a(new_n207), .b(new_n205), .c(new_n210), .o1(new_n211));
  tech160nm_fiaoi012aa1n02p5x5 g116(.a(new_n210), .b(new_n207), .c(new_n205), .o1(new_n212));
  norb02aa1n02x7               g117(.a(new_n211), .b(new_n212), .out0(\s[20] ));
  nona23aa1d18x5               g118(.a(new_n209), .b(new_n201), .c(new_n200), .d(new_n208), .out0(new_n214));
  norb02aa1n15x5               g119(.a(new_n197), .b(new_n214), .out0(new_n215));
  tech160nm_fiaoi012aa1n04x5   g120(.a(new_n208), .b(new_n200), .c(new_n209), .o1(new_n216));
  oai012aa1n12x5               g121(.a(new_n216), .b(new_n214), .c(new_n199), .o1(new_n217));
  nor042aa1n04x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  nand42aa1n03x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  norb02aa1n03x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  aoai13aa1n06x5               g125(.a(new_n220), .b(new_n217), .c(new_n181), .d(new_n215), .o1(new_n221));
  inv030aa1n02x5               g126(.a(new_n221), .o1(new_n222));
  aoi112aa1n02x5               g127(.a(new_n220), .b(new_n217), .c(new_n181), .d(new_n215), .o1(new_n223));
  norp02aa1n02x5               g128(.a(new_n222), .b(new_n223), .o1(\s[21] ));
  inv040aa1n03x5               g129(.a(new_n218), .o1(new_n225));
  xnrc02aa1n12x5               g130(.a(\b[21] ), .b(\a[22] ), .out0(new_n226));
  nand23aa1n03x5               g131(.a(new_n221), .b(new_n225), .c(new_n226), .o1(new_n227));
  aoi012aa1n02x7               g132(.a(new_n226), .b(new_n221), .c(new_n225), .o1(new_n228));
  norb02aa1n03x4               g133(.a(new_n227), .b(new_n228), .out0(\s[22] ));
  nano23aa1n06x5               g134(.a(new_n200), .b(new_n208), .c(new_n209), .d(new_n201), .out0(new_n230));
  nano32aa1n03x7               g135(.a(new_n226), .b(new_n230), .c(new_n197), .d(new_n220), .out0(new_n231));
  nano22aa1n03x7               g136(.a(new_n226), .b(new_n225), .c(new_n219), .out0(new_n232));
  oao003aa1n02x5               g137(.a(\a[22] ), .b(\b[21] ), .c(new_n225), .carry(new_n233));
  inv000aa1n02x5               g138(.a(new_n233), .o1(new_n234));
  aoi012aa1n02x5               g139(.a(new_n234), .b(new_n217), .c(new_n232), .o1(new_n235));
  inv000aa1n03x5               g140(.a(new_n235), .o1(new_n236));
  xorc02aa1n12x5               g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n236), .c(new_n181), .d(new_n231), .o1(new_n238));
  inv040aa1n03x5               g143(.a(new_n238), .o1(new_n239));
  aoi112aa1n02x5               g144(.a(new_n237), .b(new_n236), .c(new_n181), .d(new_n231), .o1(new_n240));
  norp02aa1n02x5               g145(.a(new_n239), .b(new_n240), .o1(\s[23] ));
  nor042aa1n03x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  tech160nm_fixnrc02aa1n05x5   g148(.a(\b[23] ), .b(\a[24] ), .out0(new_n244));
  nanp03aa1n03x5               g149(.a(new_n238), .b(new_n243), .c(new_n244), .o1(new_n245));
  tech160nm_fiaoi012aa1n02p5x5 g150(.a(new_n244), .b(new_n238), .c(new_n243), .o1(new_n246));
  norb02aa1n03x4               g151(.a(new_n245), .b(new_n246), .out0(\s[24] ));
  inv000aa1n02x5               g152(.a(new_n215), .o1(new_n248));
  norb02aa1n12x5               g153(.a(new_n237), .b(new_n244), .out0(new_n249));
  nano22aa1n02x4               g154(.a(new_n248), .b(new_n232), .c(new_n249), .out0(new_n250));
  inv020aa1n03x5               g155(.a(new_n216), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n232), .b(new_n251), .c(new_n230), .d(new_n206), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n249), .o1(new_n253));
  oao003aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .c(new_n243), .carry(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n253), .c(new_n252), .d(new_n233), .o1(new_n255));
  aoi012aa1n03x5               g160(.a(new_n255), .b(new_n181), .c(new_n250), .o1(new_n256));
  xorc02aa1n12x5               g161(.a(\a[25] ), .b(\b[24] ), .out0(new_n257));
  xnrc02aa1n02x5               g162(.a(new_n256), .b(new_n257), .out0(\s[25] ));
  nor042aa1n03x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  aoai13aa1n06x5               g165(.a(new_n257), .b(new_n255), .c(new_n181), .d(new_n250), .o1(new_n261));
  xnrc02aa1n12x5               g166(.a(\b[25] ), .b(\a[26] ), .out0(new_n262));
  nanp03aa1n03x5               g167(.a(new_n261), .b(new_n260), .c(new_n262), .o1(new_n263));
  tech160nm_fiaoi012aa1n04x5   g168(.a(new_n262), .b(new_n261), .c(new_n260), .o1(new_n264));
  norb02aa1n02x7               g169(.a(new_n263), .b(new_n264), .out0(\s[26] ));
  nanb02aa1n02x5               g170(.a(new_n262), .b(new_n257), .out0(new_n266));
  nano32aa1n03x7               g171(.a(new_n266), .b(new_n215), .c(new_n232), .d(new_n249), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n192), .c(new_n120), .d(new_n189), .o1(new_n268));
  inv000aa1n02x5               g173(.a(new_n266), .o1(new_n269));
  nanp02aa1n06x5               g174(.a(new_n255), .b(new_n269), .o1(new_n270));
  oao003aa1n02x5               g175(.a(\a[26] ), .b(\b[25] ), .c(new_n260), .carry(new_n271));
  xorc02aa1n12x5               g176(.a(\a[27] ), .b(\b[26] ), .out0(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  aoi013aa1n03x5               g178(.a(new_n273), .b(new_n268), .c(new_n270), .d(new_n271), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n249), .b(new_n234), .c(new_n217), .d(new_n232), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n271), .b(new_n266), .c(new_n275), .d(new_n254), .o1(new_n276));
  aoi112aa1n02x5               g181(.a(new_n276), .b(new_n272), .c(new_n181), .d(new_n267), .o1(new_n277));
  norp02aa1n02x5               g182(.a(new_n274), .b(new_n277), .o1(\s[27] ));
  nor042aa1n03x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  inv040aa1n03x5               g184(.a(new_n279), .o1(new_n280));
  tech160nm_fixnrc02aa1n04x5   g185(.a(\b[27] ), .b(\a[28] ), .out0(new_n281));
  nano22aa1n03x5               g186(.a(new_n274), .b(new_n280), .c(new_n281), .out0(new_n282));
  nand23aa1n03x5               g187(.a(new_n231), .b(new_n249), .c(new_n269), .o1(new_n283));
  oaoi13aa1n06x5               g188(.a(new_n283), .b(new_n180), .c(new_n144), .d(new_n176), .o1(new_n284));
  oaih12aa1n02x5               g189(.a(new_n272), .b(new_n276), .c(new_n284), .o1(new_n285));
  tech160nm_fiaoi012aa1n02p5x5 g190(.a(new_n281), .b(new_n285), .c(new_n280), .o1(new_n286));
  norp02aa1n03x5               g191(.a(new_n286), .b(new_n282), .o1(\s[28] ));
  tech160nm_fixnrc02aa1n04x5   g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  norb02aa1n02x5               g193(.a(new_n272), .b(new_n281), .out0(new_n289));
  oaih12aa1n02x5               g194(.a(new_n289), .b(new_n276), .c(new_n284), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n280), .carry(new_n291));
  tech160nm_fiaoi012aa1n05x5   g196(.a(new_n288), .b(new_n290), .c(new_n291), .o1(new_n292));
  inv000aa1n02x5               g197(.a(new_n289), .o1(new_n293));
  aoi013aa1n03x5               g198(.a(new_n293), .b(new_n268), .c(new_n270), .d(new_n271), .o1(new_n294));
  nano22aa1n03x5               g199(.a(new_n294), .b(new_n288), .c(new_n291), .out0(new_n295));
  norp02aa1n03x5               g200(.a(new_n292), .b(new_n295), .o1(\s[29] ));
  nanp02aa1n02x5               g201(.a(\b[0] ), .b(\a[1] ), .o1(new_n297));
  xorb03aa1n02x5               g202(.a(new_n297), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g203(.a(\b[29] ), .b(\a[30] ), .out0(new_n299));
  norb03aa1n12x5               g204(.a(new_n272), .b(new_n288), .c(new_n281), .out0(new_n300));
  oaih12aa1n02x5               g205(.a(new_n300), .b(new_n276), .c(new_n284), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .carry(new_n302));
  tech160nm_fiaoi012aa1n02p5x5 g207(.a(new_n299), .b(new_n301), .c(new_n302), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n300), .o1(new_n304));
  aoi013aa1n03x5               g209(.a(new_n304), .b(new_n268), .c(new_n270), .d(new_n271), .o1(new_n305));
  nano22aa1n03x5               g210(.a(new_n305), .b(new_n299), .c(new_n302), .out0(new_n306));
  norp02aa1n03x5               g211(.a(new_n303), .b(new_n306), .o1(\s[30] ));
  norb02aa1n02x5               g212(.a(new_n300), .b(new_n299), .out0(new_n308));
  inv000aa1n02x5               g213(.a(new_n308), .o1(new_n309));
  aoi013aa1n03x5               g214(.a(new_n309), .b(new_n268), .c(new_n270), .d(new_n271), .o1(new_n310));
  oao003aa1n03x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .carry(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[30] ), .b(\a[31] ), .out0(new_n312));
  nano22aa1n03x5               g217(.a(new_n310), .b(new_n311), .c(new_n312), .out0(new_n313));
  oaih12aa1n02x5               g218(.a(new_n308), .b(new_n276), .c(new_n284), .o1(new_n314));
  tech160nm_fiaoi012aa1n02p5x5 g219(.a(new_n312), .b(new_n314), .c(new_n311), .o1(new_n315));
  norp02aa1n03x5               g220(.a(new_n315), .b(new_n313), .o1(\s[31] ));
  norp03aa1n02x5               g221(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n317));
  norb02aa1n02x5               g222(.a(new_n137), .b(new_n317), .out0(\s[3] ));
  xorc02aa1n02x5               g223(.a(\a[4] ), .b(\b[3] ), .out0(new_n319));
  norp02aa1n02x5               g224(.a(new_n319), .b(new_n104), .o1(new_n320));
  aoi022aa1n02x5               g225(.a(new_n139), .b(new_n319), .c(new_n137), .d(new_n320), .o1(\s[4] ));
  xnbna2aa1n03x5               g226(.a(new_n113), .b(new_n139), .c(new_n114), .out0(\s[5] ));
  nanb03aa1n02x5               g227(.a(new_n113), .b(new_n139), .c(new_n114), .out0(new_n323));
  xnbna2aa1n03x5               g228(.a(new_n115), .b(new_n323), .c(new_n111), .out0(\s[6] ));
  oabi12aa1n02x5               g229(.a(new_n117), .b(new_n116), .c(new_n110), .out0(new_n325));
  xorb03aa1n02x5               g230(.a(new_n325), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g231(.a(new_n100), .b(new_n325), .c(new_n101), .o1(new_n327));
  xnrb03aa1n02x5               g232(.a(new_n327), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g233(.a(new_n120), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


