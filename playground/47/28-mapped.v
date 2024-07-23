// Benchmark "adder" written by ABC on Thu Jul 18 12:19:59 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n253, new_n254, new_n255, new_n256, new_n257, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n310, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n327, new_n328, new_n329, new_n330,
    new_n331, new_n333, new_n336, new_n337, new_n338, new_n340, new_n341,
    new_n343, new_n344, new_n346;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_fixorc02aa1n03p5x5 g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  orn002aa1n02x5               g002(.a(\a[9] ), .b(\b[8] ), .o(new_n98));
  and002aa1n12x5               g003(.a(\b[0] ), .b(\a[1] ), .o(new_n99));
  oaoi03aa1n09x5               g004(.a(\a[2] ), .b(\b[1] ), .c(new_n99), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand02aa1n04x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor002aa1n03x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand02aa1n03x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nano23aa1n06x5               g009(.a(new_n101), .b(new_n103), .c(new_n104), .d(new_n102), .out0(new_n105));
  nand42aa1n02x5               g010(.a(new_n105), .b(new_n100), .o1(new_n106));
  inv000aa1d42x5               g011(.a(\a[3] ), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\b[2] ), .o1(new_n108));
  aoai13aa1n04x5               g013(.a(new_n102), .b(new_n101), .c(new_n107), .d(new_n108), .o1(new_n109));
  nand02aa1d04x5               g014(.a(new_n106), .b(new_n109), .o1(new_n110));
  nor042aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  tech160nm_finand02aa1n05x5   g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand42aa1n16x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nor042aa1n09x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n113), .b(new_n112), .c(new_n114), .d(new_n111), .out0(new_n115));
  tech160nm_fixorc02aa1n04x5   g020(.a(\a[5] ), .b(\b[4] ), .out0(new_n116));
  norp02aa1n04x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nand42aa1d28x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  norb02aa1n02x5               g023(.a(new_n118), .b(new_n117), .out0(new_n119));
  nano22aa1n06x5               g024(.a(new_n115), .b(new_n116), .c(new_n119), .out0(new_n120));
  inv000aa1n03x5               g025(.a(new_n111), .o1(new_n121));
  inv000aa1d42x5               g026(.a(new_n113), .o1(new_n122));
  inv000aa1d42x5               g027(.a(new_n114), .o1(new_n123));
  nor042aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  aoai13aa1n04x5               g029(.a(new_n112), .b(new_n117), .c(new_n124), .d(new_n118), .o1(new_n125));
  aoai13aa1n06x5               g030(.a(new_n123), .b(new_n122), .c(new_n125), .d(new_n121), .o1(new_n126));
  tech160nm_fixorc02aa1n03p5x5 g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n06x5               g032(.a(new_n127), .b(new_n126), .c(new_n110), .d(new_n120), .o1(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n97), .b(new_n128), .c(new_n98), .out0(\s[10] ));
  oai022aa1d18x5               g034(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n130));
  nanb02aa1n03x5               g035(.a(new_n130), .b(new_n128), .out0(new_n131));
  nor002aa1d24x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  aoi022aa1d18x5               g037(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  nand42aa1n03x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n132), .b(new_n135), .out0(new_n136));
  aob012aa1n02x5               g041(.a(new_n131), .b(\b[9] ), .c(\a[10] ), .out0(new_n137));
  aoi022aa1n02x5               g042(.a(new_n137), .b(new_n136), .c(new_n131), .d(new_n134), .o1(\s[11] ));
  inv000aa1d42x5               g043(.a(\a[12] ), .o1(new_n139));
  tech160nm_fiaoi012aa1n05x5   g044(.a(new_n132), .b(new_n131), .c(new_n133), .o1(new_n140));
  xorb03aa1n02x5               g045(.a(new_n140), .b(\b[11] ), .c(new_n139), .out0(\s[12] ));
  nor002aa1n12x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanp02aa1n12x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nano23aa1n03x7               g048(.a(new_n132), .b(new_n142), .c(new_n143), .d(new_n135), .out0(new_n144));
  and003aa1n02x5               g049(.a(new_n144), .b(new_n127), .c(new_n97), .o(new_n145));
  aoai13aa1n06x5               g050(.a(new_n145), .b(new_n126), .c(new_n110), .d(new_n120), .o1(new_n146));
  norb03aa1n12x5               g051(.a(new_n143), .b(new_n132), .c(new_n142), .out0(new_n147));
  oa0012aa1n03x5               g052(.a(new_n143), .b(new_n142), .c(new_n132), .o(new_n148));
  aoi013aa1n09x5               g053(.a(new_n148), .b(new_n147), .c(new_n133), .d(new_n130), .o1(new_n149));
  tech160nm_fixnrc02aa1n05x5   g054(.a(\b[12] ), .b(\a[13] ), .out0(new_n150));
  inv040aa1n02x5               g055(.a(new_n150), .o1(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n151), .b(new_n146), .c(new_n149), .out0(\s[13] ));
  inv020aa1d32x5               g057(.a(\a[14] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n146), .b(new_n149), .o1(new_n154));
  norp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  aoi012aa1n03x5               g060(.a(new_n155), .b(new_n154), .c(new_n151), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(new_n153), .out0(\s[14] ));
  aobi12aa1n02x7               g062(.a(new_n109), .b(new_n105), .c(new_n100), .out0(new_n158));
  nano23aa1n03x5               g063(.a(new_n114), .b(new_n111), .c(new_n112), .d(new_n113), .out0(new_n159));
  nanp03aa1n02x5               g064(.a(new_n159), .b(new_n116), .c(new_n119), .o1(new_n160));
  nand42aa1n02x5               g065(.a(new_n125), .b(new_n121), .o1(new_n161));
  aoi012aa1n03x5               g066(.a(new_n114), .b(new_n161), .c(new_n113), .o1(new_n162));
  oaih12aa1n06x5               g067(.a(new_n162), .b(new_n158), .c(new_n160), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n149), .o1(new_n164));
  xnrc02aa1n12x5               g069(.a(\b[13] ), .b(\a[14] ), .out0(new_n165));
  norp02aa1n02x5               g070(.a(new_n165), .b(new_n150), .o1(new_n166));
  aoai13aa1n02x5               g071(.a(new_n166), .b(new_n164), .c(new_n163), .d(new_n145), .o1(new_n167));
  inv000aa1d42x5               g072(.a(\b[13] ), .o1(new_n168));
  oao003aa1n02x5               g073(.a(new_n153), .b(new_n168), .c(new_n155), .carry(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  nor002aa1n04x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nand22aa1n04x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  norb02aa1n06x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  xnbna2aa1n03x5               g078(.a(new_n173), .b(new_n167), .c(new_n170), .out0(\s[15] ));
  nanp02aa1n02x5               g079(.a(new_n167), .b(new_n170), .o1(new_n175));
  nor002aa1d32x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nand02aa1d20x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nanb02aa1n12x5               g082(.a(new_n176), .b(new_n177), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n171), .c(new_n175), .d(new_n172), .o1(new_n179));
  aoai13aa1n02x5               g084(.a(new_n173), .b(new_n169), .c(new_n154), .d(new_n166), .o1(new_n180));
  nona22aa1n02x4               g085(.a(new_n180), .b(new_n178), .c(new_n171), .out0(new_n181));
  nanp02aa1n02x5               g086(.a(new_n179), .b(new_n181), .o1(\s[16] ));
  nona23aa1d18x5               g087(.a(new_n151), .b(new_n173), .c(new_n165), .d(new_n178), .out0(new_n183));
  nano32aa1d12x5               g088(.a(new_n183), .b(new_n144), .c(new_n127), .d(new_n97), .out0(new_n184));
  aoai13aa1n12x5               g089(.a(new_n184), .b(new_n126), .c(new_n110), .d(new_n120), .o1(new_n185));
  inv000aa1n02x5               g090(.a(new_n171), .o1(new_n186));
  inv000aa1d42x5               g091(.a(new_n176), .o1(new_n187));
  inv000aa1d42x5               g092(.a(new_n177), .o1(new_n188));
  oai022aa1d18x5               g093(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n189));
  oai112aa1n04x5               g094(.a(new_n189), .b(new_n172), .c(new_n168), .d(new_n153), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n187), .b(new_n188), .c(new_n190), .d(new_n186), .o1(new_n191));
  oab012aa1d15x5               g096(.a(new_n191), .b(new_n183), .c(new_n149), .out0(new_n192));
  nanp02aa1n09x5               g097(.a(new_n185), .b(new_n192), .o1(new_n193));
  nor042aa1n09x5               g098(.a(\b[16] ), .b(\a[17] ), .o1(new_n194));
  nand42aa1n16x5               g099(.a(\b[16] ), .b(\a[17] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  norp02aa1n02x5               g101(.a(new_n149), .b(new_n183), .o1(new_n197));
  norp03aa1n02x5               g102(.a(new_n197), .b(new_n191), .c(new_n196), .o1(new_n198));
  aoi022aa1n02x5               g103(.a(new_n193), .b(new_n196), .c(new_n185), .d(new_n198), .o1(\s[17] ));
  inv000aa1d42x5               g104(.a(\a[18] ), .o1(new_n200));
  tech160nm_fiaoi012aa1n05x5   g105(.a(new_n194), .b(new_n193), .c(new_n196), .o1(new_n201));
  xorb03aa1n02x5               g106(.a(new_n201), .b(\b[17] ), .c(new_n200), .out0(\s[18] ));
  oabi12aa1n06x5               g107(.a(new_n191), .b(new_n149), .c(new_n183), .out0(new_n203));
  nor042aa1n06x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nand42aa1n20x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nano23aa1d12x5               g110(.a(new_n194), .b(new_n204), .c(new_n205), .d(new_n195), .out0(new_n206));
  aoai13aa1n06x5               g111(.a(new_n206), .b(new_n203), .c(new_n163), .d(new_n184), .o1(new_n207));
  oa0012aa1n02x5               g112(.a(new_n205), .b(new_n204), .c(new_n194), .o(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  nor042aa1n04x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nanp02aa1n04x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  norb02aa1d21x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n207), .c(new_n209), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g119(.a(new_n207), .b(new_n209), .o1(new_n215));
  nor042aa1n04x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nanp02aa1n06x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nanb02aa1n06x5               g122(.a(new_n216), .b(new_n217), .out0(new_n218));
  aoai13aa1n02x5               g123(.a(new_n218), .b(new_n210), .c(new_n215), .d(new_n211), .o1(new_n219));
  aoai13aa1n03x5               g124(.a(new_n212), .b(new_n208), .c(new_n193), .d(new_n206), .o1(new_n220));
  nona22aa1n02x5               g125(.a(new_n220), .b(new_n218), .c(new_n210), .out0(new_n221));
  nanp02aa1n03x5               g126(.a(new_n221), .b(new_n219), .o1(\s[20] ));
  nanb03aa1d24x5               g127(.a(new_n218), .b(new_n206), .c(new_n212), .out0(new_n223));
  nanb03aa1n06x5               g128(.a(new_n216), .b(new_n217), .c(new_n211), .out0(new_n224));
  orn002aa1n02x5               g129(.a(\a[19] ), .b(\b[18] ), .o(new_n225));
  oai112aa1n06x5               g130(.a(new_n225), .b(new_n205), .c(new_n204), .d(new_n194), .o1(new_n226));
  aoi012aa1n12x5               g131(.a(new_n216), .b(new_n210), .c(new_n217), .o1(new_n227));
  oai012aa1n18x5               g132(.a(new_n227), .b(new_n226), .c(new_n224), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoai13aa1n04x5               g134(.a(new_n229), .b(new_n223), .c(new_n185), .d(new_n192), .o1(new_n230));
  nor042aa1n04x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  nanp02aa1n02x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  norb02aa1n02x5               g137(.a(new_n232), .b(new_n231), .out0(new_n233));
  inv000aa1d42x5               g138(.a(new_n223), .o1(new_n234));
  aoi112aa1n02x5               g139(.a(new_n233), .b(new_n228), .c(new_n193), .d(new_n234), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n230), .c(new_n233), .o1(\s[21] ));
  nor042aa1n02x5               g141(.a(\b[21] ), .b(\a[22] ), .o1(new_n237));
  nand42aa1n03x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  nanb02aa1n02x5               g143(.a(new_n237), .b(new_n238), .out0(new_n239));
  aoai13aa1n03x5               g144(.a(new_n239), .b(new_n231), .c(new_n230), .d(new_n233), .o1(new_n240));
  nand02aa1n02x5               g145(.a(new_n230), .b(new_n233), .o1(new_n241));
  nona22aa1n02x5               g146(.a(new_n241), .b(new_n239), .c(new_n231), .out0(new_n242));
  nanp02aa1n03x5               g147(.a(new_n242), .b(new_n240), .o1(\s[22] ));
  nano23aa1n09x5               g148(.a(new_n231), .b(new_n237), .c(new_n238), .d(new_n232), .out0(new_n244));
  nanb02aa1n02x5               g149(.a(new_n223), .b(new_n244), .out0(new_n245));
  aoi012aa1n02x5               g150(.a(new_n245), .b(new_n185), .c(new_n192), .o1(new_n246));
  oa0012aa1n02x5               g151(.a(new_n238), .b(new_n237), .c(new_n231), .o(new_n247));
  aoi012aa1n02x5               g152(.a(new_n247), .b(new_n228), .c(new_n244), .o1(new_n248));
  aoai13aa1n04x5               g153(.a(new_n248), .b(new_n245), .c(new_n185), .d(new_n192), .o1(new_n249));
  xorc02aa1n12x5               g154(.a(\a[23] ), .b(\b[22] ), .out0(new_n250));
  aoi112aa1n02x5               g155(.a(new_n250), .b(new_n247), .c(new_n228), .d(new_n244), .o1(new_n251));
  aboi22aa1n03x5               g156(.a(new_n246), .b(new_n251), .c(new_n249), .d(new_n250), .out0(\s[23] ));
  norp02aa1n02x5               g157(.a(\b[22] ), .b(\a[23] ), .o1(new_n253));
  tech160nm_fixnrc02aa1n05x5   g158(.a(\b[23] ), .b(\a[24] ), .out0(new_n254));
  aoai13aa1n03x5               g159(.a(new_n254), .b(new_n253), .c(new_n249), .d(new_n250), .o1(new_n255));
  nand42aa1n02x5               g160(.a(new_n249), .b(new_n250), .o1(new_n256));
  nona22aa1n03x5               g161(.a(new_n256), .b(new_n254), .c(new_n253), .out0(new_n257));
  nanp02aa1n03x5               g162(.a(new_n257), .b(new_n255), .o1(\s[24] ));
  norb02aa1n03x5               g163(.a(new_n250), .b(new_n254), .out0(new_n259));
  nano22aa1n03x7               g164(.a(new_n223), .b(new_n259), .c(new_n244), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n203), .c(new_n163), .d(new_n184), .o1(new_n261));
  nano22aa1n02x4               g166(.a(new_n216), .b(new_n211), .c(new_n217), .out0(new_n262));
  oai012aa1n02x5               g167(.a(new_n205), .b(\b[18] ), .c(\a[19] ), .o1(new_n263));
  oab012aa1n02x4               g168(.a(new_n263), .b(new_n194), .c(new_n204), .out0(new_n264));
  inv040aa1n03x5               g169(.a(new_n227), .o1(new_n265));
  aoai13aa1n06x5               g170(.a(new_n244), .b(new_n265), .c(new_n264), .d(new_n262), .o1(new_n266));
  inv020aa1n02x5               g171(.a(new_n247), .o1(new_n267));
  inv000aa1n02x5               g172(.a(new_n259), .o1(new_n268));
  oai022aa1n02x5               g173(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n269));
  aob012aa1n02x5               g174(.a(new_n269), .b(\b[23] ), .c(\a[24] ), .out0(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n268), .c(new_n266), .d(new_n267), .o1(new_n271));
  nanb02aa1n02x5               g176(.a(new_n271), .b(new_n261), .out0(new_n272));
  xorb03aa1n02x5               g177(.a(new_n272), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g178(.a(\b[24] ), .b(\a[25] ), .o1(new_n274));
  xorc02aa1n12x5               g179(.a(\a[25] ), .b(\b[24] ), .out0(new_n275));
  tech160nm_fixnrc02aa1n05x5   g180(.a(\b[25] ), .b(\a[26] ), .out0(new_n276));
  aoai13aa1n02x5               g181(.a(new_n276), .b(new_n274), .c(new_n272), .d(new_n275), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n275), .b(new_n271), .c(new_n193), .d(new_n260), .o1(new_n278));
  nona22aa1n02x5               g183(.a(new_n278), .b(new_n276), .c(new_n274), .out0(new_n279));
  nanp02aa1n03x5               g184(.a(new_n277), .b(new_n279), .o1(\s[26] ));
  norb02aa1n03x4               g185(.a(new_n275), .b(new_n276), .out0(new_n281));
  inv000aa1n02x5               g186(.a(new_n281), .o1(new_n282));
  nano23aa1d15x5               g187(.a(new_n282), .b(new_n223), .c(new_n259), .d(new_n244), .out0(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n203), .c(new_n163), .d(new_n184), .o1(new_n284));
  nanp02aa1n02x5               g189(.a(\b[25] ), .b(\a[26] ), .o1(new_n285));
  oai022aa1n02x5               g190(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n286));
  aoi022aa1n06x5               g191(.a(new_n271), .b(new_n281), .c(new_n285), .d(new_n286), .o1(new_n287));
  xorc02aa1n12x5               g192(.a(\a[27] ), .b(\b[26] ), .out0(new_n288));
  xnbna2aa1n03x5               g193(.a(new_n288), .b(new_n287), .c(new_n284), .out0(\s[27] ));
  nand42aa1n02x5               g194(.a(new_n287), .b(new_n284), .o1(new_n290));
  norp02aa1n02x5               g195(.a(\b[26] ), .b(\a[27] ), .o1(new_n291));
  nor002aa1n02x5               g196(.a(\b[27] ), .b(\a[28] ), .o1(new_n292));
  nanp02aa1n04x5               g197(.a(\b[27] ), .b(\a[28] ), .o1(new_n293));
  nanb02aa1n06x5               g198(.a(new_n292), .b(new_n293), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n291), .c(new_n290), .d(new_n288), .o1(new_n295));
  aoai13aa1n04x5               g200(.a(new_n259), .b(new_n247), .c(new_n228), .d(new_n244), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(new_n286), .b(new_n285), .o1(new_n297));
  aoai13aa1n04x5               g202(.a(new_n297), .b(new_n282), .c(new_n296), .d(new_n270), .o1(new_n298));
  aoai13aa1n03x5               g203(.a(new_n288), .b(new_n298), .c(new_n193), .d(new_n283), .o1(new_n299));
  nona22aa1n02x5               g204(.a(new_n299), .b(new_n294), .c(new_n291), .out0(new_n300));
  nanp02aa1n03x5               g205(.a(new_n295), .b(new_n300), .o1(\s[28] ));
  norb02aa1n15x5               g206(.a(new_n288), .b(new_n294), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n298), .c(new_n193), .d(new_n283), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n302), .o1(new_n304));
  aoi012aa1n02x5               g209(.a(new_n292), .b(new_n291), .c(new_n293), .o1(new_n305));
  aoai13aa1n03x5               g210(.a(new_n305), .b(new_n304), .c(new_n287), .d(new_n284), .o1(new_n306));
  xorc02aa1n02x5               g211(.a(\a[29] ), .b(\b[28] ), .out0(new_n307));
  norb02aa1n02x5               g212(.a(new_n305), .b(new_n307), .out0(new_n308));
  aoi022aa1n03x5               g213(.a(new_n306), .b(new_n307), .c(new_n303), .d(new_n308), .o1(\s[29] ));
  inv000aa1d42x5               g214(.a(\a[2] ), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n99), .b(\b[1] ), .c(new_n310), .out0(\s[2] ));
  nano22aa1n03x7               g216(.a(new_n294), .b(new_n288), .c(new_n307), .out0(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n298), .c(new_n193), .d(new_n283), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n312), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[29] ), .b(\b[28] ), .c(new_n305), .carry(new_n315));
  aoai13aa1n02x7               g220(.a(new_n315), .b(new_n314), .c(new_n287), .d(new_n284), .o1(new_n316));
  xorc02aa1n02x5               g221(.a(\a[30] ), .b(\b[29] ), .out0(new_n317));
  norb02aa1n02x5               g222(.a(new_n315), .b(new_n317), .out0(new_n318));
  aoi022aa1n03x5               g223(.a(new_n316), .b(new_n317), .c(new_n313), .d(new_n318), .o1(\s[30] ));
  nanp03aa1n02x5               g224(.a(new_n302), .b(new_n307), .c(new_n317), .o1(new_n320));
  nanb02aa1n03x5               g225(.a(new_n320), .b(new_n290), .out0(new_n321));
  xorc02aa1n02x5               g226(.a(\a[31] ), .b(\b[30] ), .out0(new_n322));
  oao003aa1n02x5               g227(.a(\a[30] ), .b(\b[29] ), .c(new_n315), .carry(new_n323));
  norb02aa1n02x5               g228(.a(new_n323), .b(new_n322), .out0(new_n324));
  aoai13aa1n06x5               g229(.a(new_n323), .b(new_n320), .c(new_n287), .d(new_n284), .o1(new_n325));
  aoi022aa1n03x5               g230(.a(new_n321), .b(new_n324), .c(new_n325), .d(new_n322), .o1(\s[31] ));
  inv000aa1d42x5               g231(.a(\b[1] ), .o1(new_n327));
  aoi022aa1n02x5               g232(.a(new_n327), .b(new_n310), .c(\a[1] ), .d(\b[0] ), .o1(new_n328));
  oaib12aa1n02x5               g233(.a(new_n328), .b(new_n327), .c(\a[2] ), .out0(new_n329));
  norb02aa1n02x5               g234(.a(new_n104), .b(new_n103), .out0(new_n330));
  aboi22aa1n03x5               g235(.a(new_n103), .b(new_n104), .c(new_n310), .d(new_n327), .out0(new_n331));
  aoi022aa1n02x5               g236(.a(new_n329), .b(new_n331), .c(new_n100), .d(new_n330), .o1(\s[3] ));
  oaoi03aa1n02x5               g237(.a(new_n107), .b(new_n108), .c(new_n100), .o1(new_n333));
  xnrb03aa1n02x5               g238(.a(new_n333), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnbna2aa1n03x5               g239(.a(new_n116), .b(new_n106), .c(new_n109), .out0(\s[5] ));
  aoi012aa1n02x5               g240(.a(new_n124), .b(new_n110), .c(new_n116), .o1(new_n336));
  norb03aa1n02x5               g241(.a(new_n118), .b(new_n124), .c(new_n117), .out0(new_n337));
  oaib12aa1n02x5               g242(.a(new_n337), .b(new_n158), .c(new_n116), .out0(new_n338));
  oai012aa1n02x5               g243(.a(new_n338), .b(new_n336), .c(new_n119), .o1(\s[6] ));
  aoi022aa1n02x5               g244(.a(new_n338), .b(new_n118), .c(new_n121), .d(new_n112), .o1(new_n340));
  aoi012aa1n02x5               g245(.a(new_n111), .b(\a[6] ), .c(\b[5] ), .o1(new_n341));
  aoi013aa1n02x4               g246(.a(new_n340), .b(new_n338), .c(new_n112), .d(new_n341), .o1(\s[7] ));
  norb02aa1n02x5               g247(.a(new_n113), .b(new_n114), .out0(new_n343));
  nanp03aa1n02x5               g248(.a(new_n338), .b(new_n112), .c(new_n341), .o1(new_n344));
  xnbna2aa1n03x5               g249(.a(new_n343), .b(new_n344), .c(new_n121), .out0(\s[8] ));
  aoi112aa1n02x5               g250(.a(new_n127), .b(new_n126), .c(new_n110), .d(new_n120), .o1(new_n346));
  aoi012aa1n02x5               g251(.a(new_n346), .b(new_n163), .c(new_n127), .o1(\s[9] ));
endmodule


