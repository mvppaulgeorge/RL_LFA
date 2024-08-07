// Benchmark "adder" written by ABC on Thu Jul 18 06:07:43 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n311, new_n314, new_n315, new_n317, new_n319;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand42aa1n04x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1d28x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nor042aa1d18x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  oaih12aa1n12x5               g004(.a(new_n97), .b(new_n99), .c(new_n98), .o1(new_n100));
  xnrc02aa1n12x5               g005(.a(\b[3] ), .b(\a[4] ), .out0(new_n101));
  xnrc02aa1n12x5               g006(.a(\b[2] ), .b(\a[3] ), .out0(new_n102));
  nor003aa1n06x5               g007(.a(new_n102), .b(new_n101), .c(new_n100), .o1(new_n103));
  inv000aa1d42x5               g008(.a(\a[4] ), .o1(new_n104));
  inv000aa1d42x5               g009(.a(\b[3] ), .o1(new_n105));
  nor042aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  oaoi03aa1n12x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n107));
  inv000aa1n02x5               g012(.a(new_n107), .o1(new_n108));
  tech160nm_fixnrc02aa1n05x5   g013(.a(\b[4] ), .b(\a[5] ), .out0(new_n109));
  nor002aa1d32x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n06x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand02aa1n06x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nor042aa1n06x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1d18x5               g018(.a(new_n112), .b(new_n111), .c(new_n113), .d(new_n110), .out0(new_n114));
  tech160nm_fixnrc02aa1n02p5x5 g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  nor043aa1n06x5               g020(.a(new_n114), .b(new_n115), .c(new_n109), .o1(new_n116));
  oai012aa1n06x5               g021(.a(new_n116), .b(new_n103), .c(new_n108), .o1(new_n117));
  aoi112aa1n02x7               g022(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  oaih22aa1d12x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  nano22aa1n03x7               g025(.a(new_n114), .b(new_n119), .c(new_n120), .out0(new_n121));
  nor043aa1n03x5               g026(.a(new_n121), .b(new_n118), .c(new_n110), .o1(new_n122));
  xorc02aa1n12x5               g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aobi12aa1n06x5               g028(.a(new_n123), .b(new_n117), .c(new_n122), .out0(new_n124));
  orn002aa1n18x5               g029(.a(\a[9] ), .b(\b[8] ), .o(new_n125));
  nanp02aa1n09x5               g030(.a(new_n117), .b(new_n122), .o1(new_n126));
  aobi12aa1n02x5               g031(.a(new_n125), .b(new_n126), .c(new_n123), .out0(new_n127));
  xorc02aa1n12x5               g032(.a(\a[10] ), .b(\b[9] ), .out0(new_n128));
  nand02aa1n03x5               g033(.a(new_n128), .b(new_n125), .o1(new_n129));
  oai022aa1n02x5               g034(.a(new_n127), .b(new_n128), .c(new_n129), .d(new_n124), .o1(\s[10] ));
  nand02aa1n08x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nor002aa1d24x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand02aa1d12x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n09x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  oai112aa1n03x5               g039(.a(new_n134), .b(new_n131), .c(new_n124), .d(new_n129), .o1(new_n135));
  oaoi13aa1n02x5               g040(.a(new_n134), .b(new_n131), .c(new_n124), .d(new_n129), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(\s[11] ));
  inv000aa1d42x5               g042(.a(new_n132), .o1(new_n138));
  nor002aa1n20x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand42aa1d28x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n12x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n135), .c(new_n138), .out0(\s[12] ));
  oai013aa1n03x5               g047(.a(new_n107), .b(new_n101), .c(new_n102), .d(new_n100), .o1(new_n143));
  nano23aa1n03x5               g048(.a(new_n113), .b(new_n110), .c(new_n111), .d(new_n112), .out0(new_n144));
  nanp03aa1n02x5               g049(.a(new_n144), .b(new_n119), .c(new_n120), .o1(new_n145));
  nona22aa1n03x5               g050(.a(new_n145), .b(new_n118), .c(new_n110), .out0(new_n146));
  nano23aa1n06x5               g051(.a(new_n132), .b(new_n139), .c(new_n140), .d(new_n133), .out0(new_n147));
  and003aa1n02x5               g052(.a(new_n147), .b(new_n128), .c(new_n123), .o(new_n148));
  aoai13aa1n03x5               g053(.a(new_n148), .b(new_n146), .c(new_n143), .d(new_n116), .o1(new_n149));
  nand23aa1n06x5               g054(.a(new_n134), .b(new_n141), .c(new_n131), .o1(new_n150));
  aoi012aa1n06x5               g055(.a(new_n139), .b(new_n132), .c(new_n140), .o1(new_n151));
  aoai13aa1n12x5               g056(.a(new_n151), .b(new_n150), .c(new_n125), .d(new_n128), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n149), .b(new_n153), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand42aa1n08x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n156), .b(new_n154), .c(new_n157), .o1(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1d32x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n09x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nona23aa1d18x5               g066(.a(new_n161), .b(new_n157), .c(new_n156), .d(new_n160), .out0(new_n162));
  oaih12aa1n02x5               g067(.a(new_n161), .b(new_n160), .c(new_n156), .o1(new_n163));
  aoai13aa1n04x5               g068(.a(new_n163), .b(new_n162), .c(new_n149), .d(new_n153), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  xorc02aa1n12x5               g071(.a(\a[15] ), .b(\b[14] ), .out0(new_n167));
  xnrc02aa1n12x5               g072(.a(\b[15] ), .b(\a[16] ), .out0(new_n168));
  inv040aa1n03x5               g073(.a(new_n168), .o1(new_n169));
  aoi112aa1n02x5               g074(.a(new_n169), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n170));
  aoai13aa1n03x5               g075(.a(new_n169), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n171));
  norb02aa1n02x7               g076(.a(new_n171), .b(new_n170), .out0(\s[16] ));
  nanp03aa1n03x5               g077(.a(new_n147), .b(new_n123), .c(new_n128), .o1(new_n173));
  nano23aa1n06x5               g078(.a(new_n156), .b(new_n160), .c(new_n161), .d(new_n157), .out0(new_n174));
  nano32aa1d12x5               g079(.a(new_n173), .b(new_n169), .c(new_n167), .d(new_n174), .out0(new_n175));
  aoai13aa1n06x5               g080(.a(new_n175), .b(new_n146), .c(new_n143), .d(new_n116), .o1(new_n176));
  norb03aa1n12x5               g081(.a(new_n167), .b(new_n162), .c(new_n168), .out0(new_n177));
  norb03aa1n03x5               g082(.a(new_n167), .b(new_n163), .c(new_n168), .out0(new_n178));
  inv000aa1d42x5               g083(.a(new_n166), .o1(new_n179));
  oaoi03aa1n02x5               g084(.a(\a[16] ), .b(\b[15] ), .c(new_n179), .o1(new_n180));
  aoi112aa1n06x5               g085(.a(new_n180), .b(new_n178), .c(new_n152), .d(new_n177), .o1(new_n181));
  nand02aa1d04x5               g086(.a(new_n181), .b(new_n176), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g088(.a(\a[18] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\a[17] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\b[16] ), .o1(new_n186));
  oaoi03aa1n03x5               g091(.a(new_n185), .b(new_n186), .c(new_n182), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[17] ), .c(new_n184), .out0(\s[18] ));
  nanb02aa1n02x5               g093(.a(new_n139), .b(new_n140), .out0(new_n189));
  nano32aa1n02x4               g094(.a(new_n189), .b(new_n138), .c(new_n133), .d(new_n131), .out0(new_n190));
  inv040aa1n03x5               g095(.a(new_n151), .o1(new_n191));
  aoai13aa1n06x5               g096(.a(new_n177), .b(new_n191), .c(new_n129), .d(new_n190), .o1(new_n192));
  nona22aa1n12x5               g097(.a(new_n192), .b(new_n178), .c(new_n180), .out0(new_n193));
  xroi22aa1d06x4               g098(.a(new_n185), .b(\b[16] ), .c(new_n184), .d(\b[17] ), .out0(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n193), .c(new_n126), .d(new_n175), .o1(new_n195));
  oai022aa1d24x5               g100(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n196));
  oaib12aa1n18x5               g101(.a(new_n196), .b(new_n184), .c(\b[17] ), .out0(new_n197));
  nor002aa1n20x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nand22aa1n04x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(new_n200));
  xobna2aa1n03x5               g105(.a(new_n200), .b(new_n195), .c(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g107(.a(new_n198), .o1(new_n203));
  aoi012aa1n03x5               g108(.a(new_n200), .b(new_n195), .c(new_n197), .o1(new_n204));
  nor002aa1n10x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nand22aa1n12x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanb02aa1n02x5               g111(.a(new_n205), .b(new_n206), .out0(new_n207));
  nano22aa1n03x5               g112(.a(new_n204), .b(new_n203), .c(new_n207), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n148), .b(new_n177), .o1(new_n209));
  aoi012aa1n06x5               g114(.a(new_n209), .b(new_n117), .c(new_n122), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n197), .o1(new_n211));
  oaoi13aa1n03x5               g116(.a(new_n211), .b(new_n194), .c(new_n210), .d(new_n193), .o1(new_n212));
  oaoi13aa1n03x5               g117(.a(new_n207), .b(new_n203), .c(new_n212), .d(new_n200), .o1(new_n213));
  norp02aa1n03x5               g118(.a(new_n213), .b(new_n208), .o1(\s[20] ));
  inv000aa1d42x5               g119(.a(\a[21] ), .o1(new_n215));
  nano23aa1n06x5               g120(.a(new_n198), .b(new_n205), .c(new_n206), .d(new_n199), .out0(new_n216));
  nand02aa1d04x5               g121(.a(new_n194), .b(new_n216), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  nona23aa1n03x5               g123(.a(new_n206), .b(new_n199), .c(new_n198), .d(new_n205), .out0(new_n219));
  aoi012aa1d18x5               g124(.a(new_n205), .b(new_n198), .c(new_n206), .o1(new_n220));
  oai012aa1n06x5               g125(.a(new_n220), .b(new_n219), .c(new_n197), .o1(new_n221));
  oaoi13aa1n06x5               g126(.a(new_n221), .b(new_n218), .c(new_n210), .d(new_n193), .o1(new_n222));
  xorb03aa1n03x5               g127(.a(new_n222), .b(\b[20] ), .c(new_n215), .out0(\s[21] ));
  nanb02aa1n12x5               g128(.a(\b[20] ), .b(new_n215), .out0(new_n224));
  aoai13aa1n04x5               g129(.a(new_n218), .b(new_n193), .c(new_n126), .d(new_n175), .o1(new_n225));
  tech160nm_fixnrc02aa1n05x5   g130(.a(\b[20] ), .b(\a[21] ), .out0(new_n226));
  aoib12aa1n03x5               g131(.a(new_n226), .b(new_n225), .c(new_n221), .out0(new_n227));
  tech160nm_fixnrc02aa1n04x5   g132(.a(\b[21] ), .b(\a[22] ), .out0(new_n228));
  nano22aa1n02x4               g133(.a(new_n227), .b(new_n224), .c(new_n228), .out0(new_n229));
  oaoi13aa1n03x5               g134(.a(new_n228), .b(new_n224), .c(new_n222), .d(new_n226), .o1(new_n230));
  norp02aa1n02x5               g135(.a(new_n230), .b(new_n229), .o1(\s[22] ));
  nor002aa1n04x5               g136(.a(new_n228), .b(new_n226), .o1(new_n232));
  and003aa1n02x5               g137(.a(new_n194), .b(new_n232), .c(new_n216), .o(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n193), .c(new_n126), .d(new_n175), .o1(new_n234));
  oaoi03aa1n12x5               g139(.a(\a[22] ), .b(\b[21] ), .c(new_n224), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n221), .c(new_n232), .o1(new_n236));
  xnrc02aa1n12x5               g141(.a(\b[22] ), .b(\a[23] ), .out0(new_n237));
  xobna2aa1n03x5               g142(.a(new_n237), .b(new_n234), .c(new_n236), .out0(\s[23] ));
  nor042aa1n03x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  aoi012aa1n03x5               g145(.a(new_n237), .b(new_n234), .c(new_n236), .o1(new_n241));
  xnrc02aa1n06x5               g146(.a(\b[23] ), .b(\a[24] ), .out0(new_n242));
  nano22aa1n02x4               g147(.a(new_n241), .b(new_n240), .c(new_n242), .out0(new_n243));
  inv000aa1n03x5               g148(.a(new_n236), .o1(new_n244));
  oaoi13aa1n03x5               g149(.a(new_n244), .b(new_n233), .c(new_n210), .d(new_n193), .o1(new_n245));
  oaoi13aa1n03x5               g150(.a(new_n242), .b(new_n240), .c(new_n245), .d(new_n237), .o1(new_n246));
  norp02aa1n02x5               g151(.a(new_n246), .b(new_n243), .o1(\s[24] ));
  nor042aa1n02x5               g152(.a(new_n242), .b(new_n237), .o1(new_n248));
  nano22aa1n03x7               g153(.a(new_n217), .b(new_n232), .c(new_n248), .out0(new_n249));
  inv040aa1n03x5               g154(.a(new_n220), .o1(new_n250));
  aoai13aa1n06x5               g155(.a(new_n232), .b(new_n250), .c(new_n216), .d(new_n211), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n235), .o1(new_n252));
  inv040aa1n02x5               g157(.a(new_n248), .o1(new_n253));
  oao003aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .c(new_n240), .carry(new_n254));
  aoai13aa1n04x5               g159(.a(new_n254), .b(new_n253), .c(new_n251), .d(new_n252), .o1(new_n255));
  oaoi13aa1n06x5               g160(.a(new_n255), .b(new_n249), .c(new_n210), .d(new_n193), .o1(new_n256));
  xnrb03aa1n03x5               g161(.a(new_n256), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  aoai13aa1n06x5               g164(.a(new_n249), .b(new_n193), .c(new_n126), .d(new_n175), .o1(new_n260));
  xnrc02aa1n12x5               g165(.a(\b[24] ), .b(\a[25] ), .out0(new_n261));
  aoib12aa1n06x5               g166(.a(new_n261), .b(new_n260), .c(new_n255), .out0(new_n262));
  xnrc02aa1n12x5               g167(.a(\b[25] ), .b(\a[26] ), .out0(new_n263));
  nano22aa1n03x7               g168(.a(new_n262), .b(new_n259), .c(new_n263), .out0(new_n264));
  oaoi13aa1n03x5               g169(.a(new_n263), .b(new_n259), .c(new_n256), .d(new_n261), .o1(new_n265));
  norp02aa1n03x5               g170(.a(new_n265), .b(new_n264), .o1(\s[26] ));
  nor042aa1n06x5               g171(.a(new_n263), .b(new_n261), .o1(new_n267));
  nano32aa1d12x5               g172(.a(new_n217), .b(new_n267), .c(new_n232), .d(new_n248), .out0(new_n268));
  aoai13aa1n12x5               g173(.a(new_n268), .b(new_n193), .c(new_n126), .d(new_n175), .o1(new_n269));
  oao003aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .c(new_n259), .carry(new_n270));
  aobi12aa1n06x5               g175(.a(new_n270), .b(new_n255), .c(new_n267), .out0(new_n271));
  xorc02aa1n12x5               g176(.a(\a[27] ), .b(\b[26] ), .out0(new_n272));
  xnbna2aa1n03x5               g177(.a(new_n272), .b(new_n271), .c(new_n269), .out0(\s[27] ));
  norp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  inv040aa1n03x5               g179(.a(new_n274), .o1(new_n275));
  aobi12aa1n03x5               g180(.a(new_n272), .b(new_n271), .c(new_n269), .out0(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  nano22aa1n03x5               g182(.a(new_n276), .b(new_n275), .c(new_n277), .out0(new_n278));
  aoai13aa1n03x5               g183(.a(new_n248), .b(new_n235), .c(new_n221), .d(new_n232), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n267), .o1(new_n280));
  aoai13aa1n06x5               g185(.a(new_n270), .b(new_n280), .c(new_n279), .d(new_n254), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n272), .b(new_n281), .c(new_n182), .d(new_n268), .o1(new_n282));
  tech160nm_fiaoi012aa1n02p5x5 g187(.a(new_n277), .b(new_n282), .c(new_n275), .o1(new_n283));
  norp02aa1n03x5               g188(.a(new_n283), .b(new_n278), .o1(\s[28] ));
  xnrc02aa1n02x5               g189(.a(\b[28] ), .b(\a[29] ), .out0(new_n285));
  norb02aa1n02x5               g190(.a(new_n272), .b(new_n277), .out0(new_n286));
  aoai13aa1n03x5               g191(.a(new_n286), .b(new_n281), .c(new_n182), .d(new_n268), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[28] ), .b(\b[27] ), .c(new_n275), .carry(new_n288));
  tech160nm_fiaoi012aa1n02p5x5 g193(.a(new_n285), .b(new_n287), .c(new_n288), .o1(new_n289));
  aobi12aa1n03x5               g194(.a(new_n286), .b(new_n271), .c(new_n269), .out0(new_n290));
  nano22aa1n03x5               g195(.a(new_n290), .b(new_n285), .c(new_n288), .out0(new_n291));
  norp02aa1n03x5               g196(.a(new_n289), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g198(.a(new_n272), .b(new_n285), .c(new_n277), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n281), .c(new_n182), .d(new_n268), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n288), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .out0(new_n297));
  tech160nm_fiaoi012aa1n02p5x5 g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n03x5               g203(.a(new_n294), .b(new_n271), .c(new_n269), .out0(new_n299));
  nano22aa1n03x5               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  norp02aa1n03x5               g205(.a(new_n298), .b(new_n300), .o1(\s[30] ));
  norb02aa1n02x5               g206(.a(new_n294), .b(new_n297), .out0(new_n302));
  aobi12aa1n03x5               g207(.a(new_n302), .b(new_n271), .c(new_n269), .out0(new_n303));
  oao003aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  nano22aa1n03x5               g210(.a(new_n303), .b(new_n304), .c(new_n305), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n302), .b(new_n281), .c(new_n182), .d(new_n268), .o1(new_n307));
  tech160nm_fiaoi012aa1n02p5x5 g212(.a(new_n305), .b(new_n307), .c(new_n304), .o1(new_n308));
  norp02aa1n03x5               g213(.a(new_n308), .b(new_n306), .o1(\s[31] ));
  xnrb03aa1n02x5               g214(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g215(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g217(.a(new_n143), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g218(.a(\b[4] ), .b(\a[5] ), .o1(new_n314));
  oai013aa1n02x4               g219(.a(new_n314), .b(new_n103), .c(new_n108), .d(new_n109), .o1(new_n315));
  xnrb03aa1n02x5               g220(.a(new_n315), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g221(.a(new_n119), .b(new_n115), .c(new_n315), .out0(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g223(.a(\a[7] ), .b(\b[6] ), .c(new_n317), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g225(.a(new_n123), .b(new_n117), .c(new_n122), .out0(\s[9] ));
endmodule


