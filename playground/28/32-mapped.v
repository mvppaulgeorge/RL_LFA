// Benchmark "adder" written by ABC on Thu Jul 18 02:35:43 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n327, new_n330, new_n331, new_n332, new_n334,
    new_n335, new_n337;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[6] ), .b(\a[7] ), .o1(new_n97));
  nanp02aa1n04x5               g002(.a(\b[6] ), .b(\a[7] ), .o1(new_n98));
  nanb02aa1n03x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nand42aa1n10x5               g004(.a(\b[5] ), .b(\a[6] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  nand02aa1d04x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  inv040aa1d28x5               g007(.a(\a[8] ), .o1(new_n103));
  inv040aa1d28x5               g008(.a(\b[7] ), .o1(new_n104));
  nanp02aa1n04x5               g009(.a(new_n104), .b(new_n103), .o1(new_n105));
  oaih22aa1d12x5               g010(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n106));
  nanp03aa1n02x5               g011(.a(new_n106), .b(new_n105), .c(new_n102), .o1(new_n107));
  tech160nm_fioaoi03aa1n02p5x5 g012(.a(new_n103), .b(new_n104), .c(new_n97), .o1(new_n108));
  oai013aa1n06x5               g013(.a(new_n108), .b(new_n107), .c(new_n99), .d(new_n101), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  nand22aa1n09x5               g015(.a(\b[0] ), .b(\a[1] ), .o1(new_n111));
  nor042aa1n03x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  tech160nm_fioai012aa1n05x5   g017(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n113));
  nor022aa1n06x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nor022aa1n16x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nand42aa1n03x5               g021(.a(\b[3] ), .b(\a[4] ), .o1(new_n117));
  nona23aa1n03x5               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  tech160nm_fiaoi012aa1n03p5x5 g023(.a(new_n116), .b(new_n114), .c(new_n117), .o1(new_n119));
  oai012aa1n12x5               g024(.a(new_n119), .b(new_n118), .c(new_n113), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n105), .b(new_n102), .o1(new_n121));
  tech160nm_fixnrc02aa1n02p5x5 g026(.a(\b[4] ), .b(\a[5] ), .out0(new_n122));
  nor002aa1d32x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  nona23aa1n02x4               g028(.a(new_n100), .b(new_n98), .c(new_n123), .d(new_n97), .out0(new_n124));
  nor043aa1n06x5               g029(.a(new_n124), .b(new_n122), .c(new_n121), .o1(new_n125));
  nor042aa1n03x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nand02aa1d08x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n109), .c(new_n120), .d(new_n125), .o1(new_n129));
  oai012aa1n02x5               g034(.a(new_n129), .b(\b[8] ), .c(\a[9] ), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand42aa1d28x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nor002aa1d24x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand42aa1d28x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanb02aa1n02x5               g039(.a(new_n133), .b(new_n134), .out0(new_n135));
  oai022aa1d24x5               g040(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(new_n129), .b(new_n137), .o1(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n135), .b(new_n138), .c(new_n132), .out0(\s[11] ));
  inv000aa1d42x5               g044(.a(\b[11] ), .o1(new_n140));
  nanb03aa1n12x5               g045(.a(new_n133), .b(new_n134), .c(new_n132), .out0(new_n141));
  aoib12aa1n02x5               g046(.a(new_n133), .b(new_n138), .c(new_n141), .out0(new_n142));
  xorb03aa1n02x5               g047(.a(new_n142), .b(new_n140), .c(\a[12] ), .out0(\s[12] ));
  nor042aa1n03x5               g048(.a(\b[9] ), .b(\a[10] ), .o1(new_n144));
  nor042aa1n06x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nand42aa1d28x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nano23aa1n09x5               g051(.a(new_n144), .b(new_n145), .c(new_n146), .d(new_n132), .out0(new_n147));
  nano23aa1n09x5               g052(.a(new_n126), .b(new_n133), .c(new_n134), .d(new_n127), .out0(new_n148));
  nand22aa1n09x5               g053(.a(new_n148), .b(new_n147), .o1(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  aoai13aa1n06x5               g055(.a(new_n150), .b(new_n109), .c(new_n120), .d(new_n125), .o1(new_n151));
  nanb02aa1n12x5               g056(.a(\a[12] ), .b(new_n140), .out0(new_n152));
  nand23aa1d12x5               g057(.a(new_n136), .b(new_n152), .c(new_n146), .o1(new_n153));
  tech160nm_fioai012aa1n03p5x5 g058(.a(new_n146), .b(new_n145), .c(new_n133), .o1(new_n154));
  oai012aa1n12x5               g059(.a(new_n154), .b(new_n153), .c(new_n141), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  nor042aa1d18x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nand22aa1n09x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanb02aa1n02x5               g063(.a(new_n157), .b(new_n158), .out0(new_n159));
  xobna2aa1n03x5               g064(.a(new_n159), .b(new_n151), .c(new_n156), .out0(\s[13] ));
  inv040aa1n02x5               g065(.a(new_n157), .o1(new_n161));
  aoai13aa1n02x5               g066(.a(new_n161), .b(new_n159), .c(new_n151), .d(new_n156), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n04x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nanp02aa1n04x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nona23aa1n03x5               g070(.a(new_n165), .b(new_n158), .c(new_n157), .d(new_n164), .out0(new_n166));
  oaoi03aa1n12x5               g071(.a(\a[14] ), .b(\b[13] ), .c(new_n161), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  aoai13aa1n04x5               g073(.a(new_n168), .b(new_n166), .c(new_n151), .d(new_n156), .o1(new_n169));
  xorb03aa1n02x5               g074(.a(new_n169), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nand02aa1n20x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nor042aa1n03x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nand02aa1n12x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  aoai13aa1n03x5               g080(.a(new_n175), .b(new_n171), .c(new_n169), .d(new_n172), .o1(new_n176));
  aoi112aa1n02x5               g081(.a(new_n175), .b(new_n171), .c(new_n169), .d(new_n172), .o1(new_n177));
  norb02aa1n03x4               g082(.a(new_n176), .b(new_n177), .out0(\s[16] ));
  nano23aa1n06x5               g083(.a(new_n157), .b(new_n164), .c(new_n165), .d(new_n158), .out0(new_n179));
  nano23aa1d12x5               g084(.a(new_n171), .b(new_n173), .c(new_n174), .d(new_n172), .out0(new_n180));
  nano22aa1d15x5               g085(.a(new_n149), .b(new_n179), .c(new_n180), .out0(new_n181));
  aoai13aa1n12x5               g086(.a(new_n181), .b(new_n109), .c(new_n120), .d(new_n125), .o1(new_n182));
  aoai13aa1n09x5               g087(.a(new_n180), .b(new_n167), .c(new_n155), .d(new_n179), .o1(new_n183));
  oa0012aa1n06x5               g088(.a(new_n174), .b(new_n173), .c(new_n171), .o(new_n184));
  inv000aa1d42x5               g089(.a(new_n184), .o1(new_n185));
  nand23aa1d12x5               g090(.a(new_n182), .b(new_n183), .c(new_n185), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nanp02aa1n06x5               g092(.a(\b[16] ), .b(\a[17] ), .o1(new_n188));
  nand02aa1d28x5               g093(.a(\b[17] ), .b(\a[18] ), .o1(new_n189));
  nor042aa1d18x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nanb02aa1n02x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  nor042aa1d18x5               g096(.a(\b[16] ), .b(\a[17] ), .o1(new_n192));
  nona23aa1n02x4               g097(.a(new_n182), .b(new_n183), .c(new_n184), .d(new_n192), .out0(new_n193));
  xnbna2aa1n03x5               g098(.a(new_n191), .b(new_n193), .c(new_n188), .out0(\s[18] ));
  oaoi13aa1n03x5               g099(.a(new_n166), .b(new_n154), .c(new_n153), .d(new_n141), .o1(new_n195));
  oaoi13aa1n09x5               g100(.a(new_n184), .b(new_n180), .c(new_n195), .d(new_n167), .o1(new_n196));
  nano23aa1d12x5               g101(.a(new_n190), .b(new_n192), .c(new_n188), .d(new_n189), .out0(new_n197));
  inv000aa1d42x5               g102(.a(new_n197), .o1(new_n198));
  aoi012aa1n02x5               g103(.a(new_n190), .b(new_n192), .c(new_n189), .o1(new_n199));
  aoai13aa1n04x5               g104(.a(new_n199), .b(new_n198), .c(new_n196), .d(new_n182), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  inv000aa1n02x5               g109(.a(new_n199), .o1(new_n205));
  nand42aa1n08x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  nanb02aa1n02x5               g111(.a(new_n203), .b(new_n206), .out0(new_n207));
  inv040aa1n02x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n03x5               g113(.a(new_n208), .b(new_n205), .c(new_n186), .d(new_n197), .o1(new_n209));
  xorc02aa1n12x5               g114(.a(\a[20] ), .b(\b[19] ), .out0(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  tech160nm_fiaoi012aa1n03p5x5 g116(.a(new_n211), .b(new_n209), .c(new_n204), .o1(new_n212));
  aoi112aa1n03x4               g117(.a(new_n203), .b(new_n210), .c(new_n200), .d(new_n206), .o1(new_n213));
  nor002aa1n02x5               g118(.a(new_n212), .b(new_n213), .o1(\s[20] ));
  nano22aa1d15x5               g119(.a(new_n198), .b(new_n208), .c(new_n210), .out0(new_n215));
  inv000aa1n09x5               g120(.a(new_n215), .o1(new_n216));
  aoai13aa1n12x5               g121(.a(new_n206), .b(new_n190), .c(new_n192), .d(new_n189), .o1(new_n217));
  oab012aa1d15x5               g122(.a(new_n203), .b(\a[20] ), .c(\b[19] ), .out0(new_n218));
  aoi022aa1d24x5               g123(.a(new_n217), .b(new_n218), .c(\b[19] ), .d(\a[20] ), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  aoai13aa1n04x5               g125(.a(new_n220), .b(new_n216), .c(new_n196), .d(new_n182), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1d32x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  inv040aa1n08x5               g128(.a(new_n223), .o1(new_n224));
  nanp02aa1n04x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  norb02aa1n06x5               g130(.a(new_n225), .b(new_n223), .out0(new_n226));
  aoai13aa1n03x5               g131(.a(new_n226), .b(new_n219), .c(new_n186), .d(new_n215), .o1(new_n227));
  xnrc02aa1n12x5               g132(.a(\b[21] ), .b(\a[22] ), .out0(new_n228));
  tech160nm_fiaoi012aa1n02p5x5 g133(.a(new_n228), .b(new_n227), .c(new_n224), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[22] ), .b(\b[21] ), .out0(new_n230));
  aoi112aa1n03x4               g135(.a(new_n223), .b(new_n230), .c(new_n221), .d(new_n226), .o1(new_n231));
  norp02aa1n03x5               g136(.a(new_n229), .b(new_n231), .o1(\s[22] ));
  nand42aa1n03x5               g137(.a(new_n230), .b(new_n226), .o1(new_n233));
  nona22aa1n03x5               g138(.a(new_n186), .b(new_n216), .c(new_n233), .out0(new_n234));
  xorc02aa1n12x5               g139(.a(\a[23] ), .b(\b[22] ), .out0(new_n235));
  nano22aa1n02x4               g140(.a(new_n228), .b(new_n224), .c(new_n225), .out0(new_n236));
  oaoi03aa1n02x5               g141(.a(\a[22] ), .b(\b[21] ), .c(new_n224), .o1(new_n237));
  aoi012aa1n02x5               g142(.a(new_n237), .b(new_n219), .c(new_n236), .o1(new_n238));
  xnbna2aa1n06x5               g143(.a(new_n235), .b(new_n234), .c(new_n238), .out0(\s[23] ));
  and002aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o(new_n240));
  xnrc02aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .out0(new_n241));
  aoi112aa1n02x5               g146(.a(new_n237), .b(new_n241), .c(new_n219), .d(new_n236), .o1(new_n242));
  xorc02aa1n12x5               g147(.a(\a[24] ), .b(\b[23] ), .out0(new_n243));
  aoai13aa1n02x7               g148(.a(new_n243), .b(new_n240), .c(new_n234), .d(new_n242), .o1(new_n244));
  nand42aa1n02x5               g149(.a(new_n234), .b(new_n242), .o1(new_n245));
  nona22aa1n03x5               g150(.a(new_n245), .b(new_n243), .c(new_n240), .out0(new_n246));
  nanp02aa1n03x5               g151(.a(new_n246), .b(new_n244), .o1(\s[24] ));
  nanb02aa1n02x5               g152(.a(new_n223), .b(new_n225), .out0(new_n248));
  nona23aa1n06x5               g153(.a(new_n235), .b(new_n243), .c(new_n228), .d(new_n248), .out0(new_n249));
  nano32aa1n03x7               g154(.a(new_n249), .b(new_n210), .c(new_n208), .d(new_n197), .out0(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  orn002aa1n02x5               g156(.a(\a[23] ), .b(\b[22] ), .o(new_n252));
  oaoi03aa1n02x5               g157(.a(\a[24] ), .b(\b[23] ), .c(new_n252), .o1(new_n253));
  aoi013aa1n06x4               g158(.a(new_n253), .b(new_n237), .c(new_n243), .d(new_n235), .o1(new_n254));
  oaib12aa1n18x5               g159(.a(new_n254), .b(new_n249), .c(new_n219), .out0(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  aoai13aa1n04x5               g161(.a(new_n256), .b(new_n251), .c(new_n196), .d(new_n182), .o1(new_n257));
  xorb03aa1n02x5               g162(.a(new_n257), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  xorc02aa1n02x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  aoai13aa1n03x5               g166(.a(new_n261), .b(new_n255), .c(new_n186), .d(new_n250), .o1(new_n262));
  xorc02aa1n12x5               g167(.a(\a[26] ), .b(\b[25] ), .out0(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  tech160nm_fiaoi012aa1n03p5x5 g169(.a(new_n264), .b(new_n262), .c(new_n260), .o1(new_n265));
  aoi112aa1n03x4               g170(.a(new_n259), .b(new_n263), .c(new_n257), .d(new_n261), .o1(new_n266));
  nor042aa1n03x5               g171(.a(new_n265), .b(new_n266), .o1(\s[26] ));
  nano22aa1n03x7               g172(.a(new_n233), .b(new_n235), .c(new_n243), .out0(new_n268));
  and002aa1n02x5               g173(.a(new_n263), .b(new_n261), .o(new_n269));
  nano22aa1n03x7               g174(.a(new_n216), .b(new_n268), .c(new_n269), .out0(new_n270));
  inv020aa1n02x5               g175(.a(new_n270), .o1(new_n271));
  nanp02aa1n02x5               g176(.a(\b[25] ), .b(\a[26] ), .o1(new_n272));
  oai022aa1n02x5               g177(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n273));
  aoi022aa1n06x5               g178(.a(new_n255), .b(new_n269), .c(new_n272), .d(new_n273), .o1(new_n274));
  aoai13aa1n04x5               g179(.a(new_n274), .b(new_n271), .c(new_n196), .d(new_n182), .o1(new_n275));
  xorb03aa1n03x5               g180(.a(new_n275), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nanp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  xorc02aa1n02x5               g182(.a(\a[28] ), .b(\b[27] ), .out0(new_n278));
  nor042aa1d18x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  nanp02aa1n02x5               g184(.a(new_n268), .b(new_n219), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(new_n263), .b(new_n261), .o1(new_n281));
  nanp02aa1n02x5               g186(.a(new_n273), .b(new_n272), .o1(new_n282));
  aoai13aa1n06x5               g187(.a(new_n282), .b(new_n281), .c(new_n280), .d(new_n254), .o1(new_n283));
  aoi112aa1n03x4               g188(.a(new_n279), .b(new_n283), .c(new_n186), .d(new_n270), .o1(new_n284));
  nano22aa1n03x7               g189(.a(new_n284), .b(new_n277), .c(new_n278), .out0(new_n285));
  nano23aa1n02x4               g190(.a(new_n99), .b(new_n121), .c(new_n106), .d(new_n100), .out0(new_n286));
  norb02aa1n02x5               g191(.a(new_n108), .b(new_n286), .out0(new_n287));
  nanp02aa1n02x5               g192(.a(new_n120), .b(new_n125), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n181), .o1(new_n289));
  aoi012aa1n02x5               g194(.a(new_n289), .b(new_n288), .c(new_n287), .o1(new_n290));
  nanp02aa1n02x5               g195(.a(new_n183), .b(new_n185), .o1(new_n291));
  oai012aa1n02x5               g196(.a(new_n270), .b(new_n291), .c(new_n290), .o1(new_n292));
  nona22aa1n02x4               g197(.a(new_n292), .b(new_n283), .c(new_n279), .out0(new_n293));
  aoi012aa1n02x5               g198(.a(new_n278), .b(new_n293), .c(new_n277), .o1(new_n294));
  nor002aa1n02x5               g199(.a(new_n294), .b(new_n285), .o1(\s[28] ));
  nano22aa1n02x4               g200(.a(new_n279), .b(new_n278), .c(new_n277), .out0(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n283), .c(new_n186), .d(new_n270), .o1(new_n297));
  inv030aa1d32x5               g202(.a(\a[28] ), .o1(new_n298));
  inv020aa1d32x5               g203(.a(\b[27] ), .o1(new_n299));
  oaoi03aa1n12x5               g204(.a(new_n298), .b(new_n299), .c(new_n279), .o1(new_n300));
  xorc02aa1n12x5               g205(.a(\a[29] ), .b(\b[28] ), .out0(new_n301));
  inv000aa1d42x5               g206(.a(new_n301), .o1(new_n302));
  tech160nm_fiaoi012aa1n03p5x5 g207(.a(new_n302), .b(new_n297), .c(new_n300), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n300), .o1(new_n304));
  aoi112aa1n03x4               g209(.a(new_n301), .b(new_n304), .c(new_n275), .d(new_n296), .o1(new_n305));
  nor002aa1n02x5               g210(.a(new_n303), .b(new_n305), .o1(\s[29] ));
  xorb03aa1n02x5               g211(.a(new_n111), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g212(.a(new_n302), .b(new_n279), .c(new_n278), .d(new_n277), .out0(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n283), .c(new_n186), .d(new_n270), .o1(new_n309));
  tech160nm_fioaoi03aa1n03p5x5 g214(.a(\a[29] ), .b(\b[28] ), .c(new_n300), .o1(new_n310));
  inv040aa1n03x5               g215(.a(new_n310), .o1(new_n311));
  xorc02aa1n12x5               g216(.a(\a[30] ), .b(\b[29] ), .out0(new_n312));
  inv000aa1d42x5               g217(.a(new_n312), .o1(new_n313));
  tech160nm_fiaoi012aa1n03p5x5 g218(.a(new_n313), .b(new_n309), .c(new_n311), .o1(new_n314));
  aoi112aa1n03x4               g219(.a(new_n312), .b(new_n310), .c(new_n275), .d(new_n308), .o1(new_n315));
  nor002aa1n02x5               g220(.a(new_n314), .b(new_n315), .o1(\s[30] ));
  and003aa1n02x5               g221(.a(new_n296), .b(new_n312), .c(new_n301), .o(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n283), .c(new_n186), .d(new_n270), .o1(new_n318));
  tech160nm_fioaoi03aa1n04x5   g223(.a(\a[30] ), .b(\b[29] ), .c(new_n311), .o1(new_n319));
  inv000aa1d42x5               g224(.a(new_n319), .o1(new_n320));
  xorc02aa1n02x5               g225(.a(\a[31] ), .b(\b[30] ), .out0(new_n321));
  inv000aa1d42x5               g226(.a(new_n321), .o1(new_n322));
  tech160nm_fiaoi012aa1n03p5x5 g227(.a(new_n322), .b(new_n318), .c(new_n320), .o1(new_n323));
  aoi112aa1n03x4               g228(.a(new_n321), .b(new_n319), .c(new_n275), .d(new_n317), .o1(new_n324));
  nor042aa1n03x5               g229(.a(new_n323), .b(new_n324), .o1(\s[31] ));
  xnrb03aa1n02x5               g230(.a(new_n113), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g231(.a(\a[3] ), .b(\b[2] ), .c(new_n113), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g233(.a(new_n120), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g234(.a(new_n123), .o1(new_n330));
  nanp02aa1n02x5               g235(.a(\b[4] ), .b(\a[5] ), .o1(new_n331));
  tech160nm_fioai012aa1n05x5   g236(.a(new_n331), .b(new_n120), .c(new_n122), .o1(new_n332));
  xnbna2aa1n03x5               g237(.a(new_n332), .b(new_n100), .c(new_n330), .out0(\s[6] ));
  oaoi13aa1n02x5               g238(.a(new_n99), .b(new_n330), .c(new_n332), .d(new_n101), .o1(new_n334));
  oai112aa1n02x5               g239(.a(new_n99), .b(new_n330), .c(new_n332), .d(new_n101), .o1(new_n335));
  norb02aa1n02x5               g240(.a(new_n335), .b(new_n334), .out0(\s[7] ));
  norp02aa1n03x5               g241(.a(new_n334), .b(new_n97), .o1(new_n337));
  xnbna2aa1n03x5               g242(.a(new_n337), .b(new_n102), .c(new_n105), .out0(\s[8] ));
  xnbna2aa1n03x5               g243(.a(new_n128), .b(new_n288), .c(new_n287), .out0(\s[9] ));
endmodule


