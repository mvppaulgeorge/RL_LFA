// Benchmark "adder" written by ABC on Thu Jul 18 03:15:03 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n146, new_n147, new_n148, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n158,
    new_n159, new_n160, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n179, new_n180, new_n181,
    new_n182, new_n183, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n213, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n230,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n304, new_n305, new_n306, new_n309, new_n310, new_n312, new_n314;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  inv000aa1d42x5               g002(.a(\a[2] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[1] ), .o1(new_n99));
  nand02aa1d12x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  oao003aa1n03x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .carry(new_n101));
  tech160nm_fixorc02aa1n02p5x5 g006(.a(\a[4] ), .b(\b[3] ), .out0(new_n102));
  tech160nm_fixorc02aa1n02p5x5 g007(.a(\a[3] ), .b(\b[2] ), .out0(new_n103));
  nand23aa1n04x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  orn002aa1n02x5               g009(.a(\a[4] ), .b(\b[3] ), .o(new_n105));
  aoi112aa1n02x7               g010(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n106));
  norb02aa1n03x5               g011(.a(new_n105), .b(new_n106), .out0(new_n107));
  xnrc02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .out0(new_n108));
  xorc02aa1n12x5               g013(.a(\a[5] ), .b(\b[4] ), .out0(new_n109));
  xorc02aa1n12x5               g014(.a(\a[8] ), .b(\b[7] ), .out0(new_n110));
  nand42aa1n16x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nor002aa1d32x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanb02aa1n12x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  nona23aa1n09x5               g018(.a(new_n109), .b(new_n110), .c(new_n108), .d(new_n113), .out0(new_n114));
  tech160nm_finand02aa1n05x5   g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nano22aa1n02x5               g020(.a(new_n112), .b(new_n115), .c(new_n111), .out0(new_n116));
  oai022aa1n02x5               g021(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n117));
  inv000aa1d42x5               g022(.a(new_n112), .o1(new_n118));
  oaoi03aa1n02x5               g023(.a(\a[8] ), .b(\b[7] ), .c(new_n118), .o1(new_n119));
  aoi013aa1n06x4               g024(.a(new_n119), .b(new_n116), .c(new_n110), .d(new_n117), .o1(new_n120));
  aoai13aa1n12x5               g025(.a(new_n120), .b(new_n114), .c(new_n104), .d(new_n107), .o1(new_n121));
  tech160nm_fixorc02aa1n02p5x5 g026(.a(\a[9] ), .b(\b[8] ), .out0(new_n122));
  nanp02aa1n02x5               g027(.a(new_n121), .b(new_n122), .o1(new_n123));
  xorc02aa1n02x5               g028(.a(\a[10] ), .b(\b[9] ), .out0(new_n124));
  xnbna2aa1n03x5               g029(.a(new_n124), .b(new_n123), .c(new_n97), .out0(\s[10] ));
  xnrc02aa1n02x5               g030(.a(\b[10] ), .b(\a[11] ), .out0(new_n126));
  nanp02aa1n03x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  oai022aa1d18x5               g032(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n128));
  aoai13aa1n02x5               g033(.a(new_n127), .b(new_n128), .c(new_n121), .d(new_n122), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n127), .b(new_n126), .out0(new_n130));
  aoai13aa1n06x5               g035(.a(new_n130), .b(new_n128), .c(new_n121), .d(new_n122), .o1(new_n131));
  aobi12aa1n02x5               g036(.a(new_n131), .b(new_n129), .c(new_n126), .out0(\s[11] ));
  nor042aa1n09x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  xorc02aa1n02x5               g039(.a(\a[12] ), .b(\b[11] ), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n131), .c(new_n134), .out0(\s[12] ));
  nano32aa1n06x5               g041(.a(new_n126), .b(new_n135), .c(new_n122), .d(new_n124), .out0(new_n137));
  inv000aa1d42x5               g042(.a(\a[12] ), .o1(new_n138));
  aoi022aa1d24x5               g043(.a(\b[11] ), .b(\a[12] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n139));
  aoai13aa1n04x5               g044(.a(new_n139), .b(new_n133), .c(new_n128), .d(new_n127), .o1(new_n140));
  oaib12aa1n03x5               g045(.a(new_n140), .b(\b[11] ), .c(new_n138), .out0(new_n141));
  tech160nm_fixorc02aa1n05x5   g046(.a(\a[13] ), .b(\b[12] ), .out0(new_n142));
  aoai13aa1n06x5               g047(.a(new_n142), .b(new_n141), .c(new_n121), .d(new_n137), .o1(new_n143));
  aoi112aa1n02x5               g048(.a(new_n142), .b(new_n141), .c(new_n121), .d(new_n137), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n143), .b(new_n144), .out0(\s[13] ));
  inv000aa1d42x5               g050(.a(\a[13] ), .o1(new_n146));
  inv000aa1d42x5               g051(.a(\b[12] ), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(new_n147), .b(new_n146), .o1(new_n148));
  nor002aa1n03x5               g053(.a(\b[13] ), .b(\a[14] ), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(\b[13] ), .b(\a[14] ), .o1(new_n150));
  norb02aa1n06x4               g055(.a(new_n150), .b(new_n149), .out0(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n151), .b(new_n143), .c(new_n148), .out0(\s[14] ));
  and002aa1n02x5               g057(.a(new_n142), .b(new_n151), .o(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n141), .c(new_n121), .d(new_n137), .o1(new_n154));
  aoai13aa1n06x5               g059(.a(new_n150), .b(new_n149), .c(new_n146), .d(new_n147), .o1(new_n155));
  xorc02aa1n02x5               g060(.a(\a[15] ), .b(\b[14] ), .out0(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n154), .c(new_n155), .out0(\s[15] ));
  aob012aa1n03x5               g062(.a(new_n156), .b(new_n154), .c(new_n155), .out0(new_n158));
  norp02aa1n02x5               g063(.a(\b[15] ), .b(\a[16] ), .o1(new_n159));
  nand42aa1n03x5               g064(.a(\b[15] ), .b(\a[16] ), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n160), .b(new_n159), .out0(new_n161));
  orn002aa1n06x5               g066(.a(\a[15] ), .b(\b[14] ), .o(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  nand42aa1n08x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n162), .b(new_n165), .c(new_n154), .d(new_n155), .o1(new_n166));
  aoi022aa1n03x5               g071(.a(new_n166), .b(new_n161), .c(new_n158), .d(new_n163), .o1(\s[16] ));
  inv000aa1d42x5               g072(.a(\a[11] ), .o1(new_n168));
  xroi22aa1d04x5               g073(.a(new_n168), .b(\b[10] ), .c(new_n138), .d(\b[11] ), .out0(new_n169));
  nano32aa1n03x7               g074(.a(new_n159), .b(new_n162), .c(new_n160), .d(new_n164), .out0(new_n170));
  nand23aa1n03x5               g075(.a(new_n170), .b(new_n142), .c(new_n151), .o1(new_n171));
  nano32aa1d12x5               g076(.a(new_n171), .b(new_n169), .c(new_n124), .d(new_n122), .out0(new_n172));
  nanp02aa1n06x5               g077(.a(new_n121), .b(new_n172), .o1(new_n173));
  oai122aa1n02x7               g078(.a(new_n162), .b(new_n155), .c(new_n165), .d(\b[15] ), .e(\a[16] ), .o1(new_n174));
  nanp02aa1n02x5               g079(.a(new_n174), .b(new_n160), .o1(new_n175));
  oaib12aa1n12x5               g080(.a(new_n175), .b(new_n171), .c(new_n141), .out0(new_n176));
  nanb02aa1n06x5               g081(.a(new_n176), .b(new_n173), .out0(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g083(.a(\a[17] ), .o1(new_n179));
  nanb02aa1n02x5               g084(.a(\b[16] ), .b(new_n179), .out0(new_n180));
  xorc02aa1n02x5               g085(.a(\a[17] ), .b(\b[16] ), .out0(new_n181));
  aoai13aa1n03x5               g086(.a(new_n181), .b(new_n176), .c(new_n121), .d(new_n172), .o1(new_n182));
  xorc02aa1n02x5               g087(.a(\a[18] ), .b(\b[17] ), .out0(new_n183));
  xnbna2aa1n03x5               g088(.a(new_n183), .b(new_n182), .c(new_n180), .out0(\s[18] ));
  inv000aa1d42x5               g089(.a(\a[18] ), .o1(new_n185));
  xroi22aa1d04x5               g090(.a(new_n179), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n186));
  aoai13aa1n06x5               g091(.a(new_n186), .b(new_n176), .c(new_n121), .d(new_n172), .o1(new_n187));
  oaih22aa1n04x5               g092(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n188));
  oaib12aa1n06x5               g093(.a(new_n188), .b(new_n185), .c(\b[17] ), .out0(new_n189));
  nor042aa1d18x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  nanp02aa1n06x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  nanb02aa1n02x5               g096(.a(new_n190), .b(new_n191), .out0(new_n192));
  inv000aa1d42x5               g097(.a(new_n192), .o1(new_n193));
  xnbna2aa1n03x5               g098(.a(new_n193), .b(new_n187), .c(new_n189), .out0(\s[19] ));
  xnrc02aa1n02x5               g099(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aob012aa1n02x5               g100(.a(new_n193), .b(new_n187), .c(new_n189), .out0(new_n196));
  nor002aa1n06x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  nand22aa1n12x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  aoib12aa1n02x5               g104(.a(new_n190), .b(new_n198), .c(new_n197), .out0(new_n200));
  inv000aa1d42x5               g105(.a(new_n190), .o1(new_n201));
  aoai13aa1n03x5               g106(.a(new_n201), .b(new_n192), .c(new_n187), .d(new_n189), .o1(new_n202));
  aoi022aa1n02x5               g107(.a(new_n202), .b(new_n199), .c(new_n196), .d(new_n200), .o1(\s[20] ));
  nona23aa1d18x5               g108(.a(new_n198), .b(new_n191), .c(new_n190), .d(new_n197), .out0(new_n204));
  nano22aa1n03x7               g109(.a(new_n204), .b(new_n181), .c(new_n183), .out0(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n176), .c(new_n121), .d(new_n172), .o1(new_n206));
  tech160nm_fiaoi012aa1n05x5   g111(.a(new_n197), .b(new_n190), .c(new_n198), .o1(new_n207));
  oaih12aa1n12x5               g112(.a(new_n207), .b(new_n204), .c(new_n189), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  xnrc02aa1n12x5               g114(.a(\b[20] ), .b(\a[21] ), .out0(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  xnbna2aa1n03x5               g116(.a(new_n211), .b(new_n206), .c(new_n209), .out0(\s[21] ));
  aob012aa1n02x5               g117(.a(new_n211), .b(new_n206), .c(new_n209), .out0(new_n213));
  tech160nm_fixnrc02aa1n04x5   g118(.a(\b[21] ), .b(\a[22] ), .out0(new_n214));
  nor042aa1n06x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n214), .b(new_n215), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n215), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n217), .b(new_n210), .c(new_n206), .d(new_n209), .o1(new_n218));
  aboi22aa1n03x5               g123(.a(new_n214), .b(new_n218), .c(new_n213), .d(new_n216), .out0(\s[22] ));
  nano23aa1n09x5               g124(.a(new_n190), .b(new_n197), .c(new_n198), .d(new_n191), .out0(new_n220));
  nor042aa1n06x5               g125(.a(new_n214), .b(new_n210), .o1(new_n221));
  and003aa1n02x5               g126(.a(new_n186), .b(new_n221), .c(new_n220), .o(new_n222));
  aoai13aa1n06x5               g127(.a(new_n222), .b(new_n176), .c(new_n121), .d(new_n172), .o1(new_n223));
  oao003aa1n12x5               g128(.a(\a[22] ), .b(\b[21] ), .c(new_n217), .carry(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  aoi012aa1n02x5               g130(.a(new_n225), .b(new_n208), .c(new_n221), .o1(new_n226));
  xnrc02aa1n12x5               g131(.a(\b[22] ), .b(\a[23] ), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  xnbna2aa1n03x5               g133(.a(new_n228), .b(new_n223), .c(new_n226), .out0(\s[23] ));
  aob012aa1n02x5               g134(.a(new_n228), .b(new_n223), .c(new_n226), .out0(new_n230));
  tech160nm_fixorc02aa1n02p5x5 g135(.a(\a[24] ), .b(\b[23] ), .out0(new_n231));
  nor042aa1n06x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  norp02aa1n02x5               g137(.a(new_n231), .b(new_n232), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n232), .o1(new_n234));
  aoai13aa1n02x5               g139(.a(new_n234), .b(new_n227), .c(new_n223), .d(new_n226), .o1(new_n235));
  aoi022aa1n02x5               g140(.a(new_n235), .b(new_n231), .c(new_n230), .d(new_n233), .o1(\s[24] ));
  norb02aa1n06x4               g141(.a(new_n231), .b(new_n227), .out0(new_n237));
  inv020aa1n02x5               g142(.a(new_n237), .o1(new_n238));
  nano32aa1n02x4               g143(.a(new_n238), .b(new_n186), .c(new_n221), .d(new_n220), .out0(new_n239));
  aoai13aa1n06x5               g144(.a(new_n239), .b(new_n176), .c(new_n121), .d(new_n172), .o1(new_n240));
  oaoi03aa1n02x5               g145(.a(\a[18] ), .b(\b[17] ), .c(new_n180), .o1(new_n241));
  inv030aa1n02x5               g146(.a(new_n207), .o1(new_n242));
  aoai13aa1n06x5               g147(.a(new_n221), .b(new_n242), .c(new_n220), .d(new_n241), .o1(new_n243));
  oao003aa1n02x5               g148(.a(\a[24] ), .b(\b[23] ), .c(new_n234), .carry(new_n244));
  aoai13aa1n12x5               g149(.a(new_n244), .b(new_n238), .c(new_n243), .d(new_n224), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  xorc02aa1n12x5               g151(.a(\a[25] ), .b(\b[24] ), .out0(new_n247));
  xnbna2aa1n03x5               g152(.a(new_n247), .b(new_n240), .c(new_n246), .out0(\s[25] ));
  aob012aa1n03x5               g153(.a(new_n247), .b(new_n240), .c(new_n246), .out0(new_n249));
  xorc02aa1n02x5               g154(.a(\a[26] ), .b(\b[25] ), .out0(new_n250));
  nor042aa1n03x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  norp02aa1n02x5               g156(.a(new_n250), .b(new_n251), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n251), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n247), .o1(new_n254));
  aoai13aa1n02x5               g159(.a(new_n253), .b(new_n254), .c(new_n240), .d(new_n246), .o1(new_n255));
  aoi022aa1n02x5               g160(.a(new_n255), .b(new_n250), .c(new_n249), .d(new_n252), .o1(\s[26] ));
  and002aa1n12x5               g161(.a(new_n250), .b(new_n247), .o(new_n257));
  inv000aa1n02x5               g162(.a(new_n257), .o1(new_n258));
  nano32aa1n03x7               g163(.a(new_n258), .b(new_n205), .c(new_n221), .d(new_n237), .out0(new_n259));
  aoai13aa1n09x5               g164(.a(new_n259), .b(new_n176), .c(new_n121), .d(new_n172), .o1(new_n260));
  oao003aa1n02x5               g165(.a(\a[26] ), .b(\b[25] ), .c(new_n253), .carry(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  aoi012aa1n12x5               g167(.a(new_n262), .b(new_n245), .c(new_n257), .o1(new_n263));
  xorc02aa1n12x5               g168(.a(\a[27] ), .b(\b[26] ), .out0(new_n264));
  xnbna2aa1n03x5               g169(.a(new_n264), .b(new_n263), .c(new_n260), .out0(\s[27] ));
  aoai13aa1n06x5               g170(.a(new_n237), .b(new_n225), .c(new_n208), .d(new_n221), .o1(new_n266));
  aoai13aa1n06x5               g171(.a(new_n261), .b(new_n258), .c(new_n266), .d(new_n244), .o1(new_n267));
  aoai13aa1n02x7               g172(.a(new_n264), .b(new_n267), .c(new_n177), .d(new_n259), .o1(new_n268));
  tech160nm_fixorc02aa1n02p5x5 g173(.a(\a[28] ), .b(\b[27] ), .out0(new_n269));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  norp02aa1n02x5               g175(.a(new_n269), .b(new_n270), .o1(new_n271));
  inv000aa1n03x5               g176(.a(new_n270), .o1(new_n272));
  inv000aa1n02x5               g177(.a(new_n264), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n272), .b(new_n273), .c(new_n263), .d(new_n260), .o1(new_n274));
  aoi022aa1n03x5               g179(.a(new_n274), .b(new_n269), .c(new_n268), .d(new_n271), .o1(\s[28] ));
  and002aa1n02x5               g180(.a(new_n269), .b(new_n264), .o(new_n276));
  aoai13aa1n02x7               g181(.a(new_n276), .b(new_n267), .c(new_n177), .d(new_n259), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n276), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[28] ), .b(\b[27] ), .c(new_n272), .carry(new_n279));
  aoai13aa1n03x5               g184(.a(new_n279), .b(new_n278), .c(new_n263), .d(new_n260), .o1(new_n280));
  xorc02aa1n02x5               g185(.a(\a[29] ), .b(\b[28] ), .out0(new_n281));
  norb02aa1n02x5               g186(.a(new_n279), .b(new_n281), .out0(new_n282));
  aoi022aa1n03x5               g187(.a(new_n280), .b(new_n281), .c(new_n277), .d(new_n282), .o1(\s[29] ));
  xorb03aa1n02x5               g188(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g189(.a(new_n273), .b(new_n269), .c(new_n281), .out0(new_n285));
  aoai13aa1n02x7               g190(.a(new_n285), .b(new_n267), .c(new_n177), .d(new_n259), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n285), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[29] ), .b(\b[28] ), .c(new_n279), .carry(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n287), .c(new_n263), .d(new_n260), .o1(new_n289));
  xorc02aa1n02x5               g194(.a(\a[30] ), .b(\b[29] ), .out0(new_n290));
  norb02aa1n02x5               g195(.a(new_n288), .b(new_n290), .out0(new_n291));
  aoi022aa1n03x5               g196(.a(new_n289), .b(new_n290), .c(new_n286), .d(new_n291), .o1(\s[30] ));
  nano32aa1n06x5               g197(.a(new_n273), .b(new_n290), .c(new_n269), .d(new_n281), .out0(new_n293));
  aoai13aa1n02x7               g198(.a(new_n293), .b(new_n267), .c(new_n177), .d(new_n259), .o1(new_n294));
  xorc02aa1n02x5               g199(.a(\a[31] ), .b(\b[30] ), .out0(new_n295));
  and002aa1n02x5               g200(.a(\b[29] ), .b(\a[30] ), .o(new_n296));
  oabi12aa1n02x5               g201(.a(new_n295), .b(\a[30] ), .c(\b[29] ), .out0(new_n297));
  oab012aa1n02x4               g202(.a(new_n297), .b(new_n288), .c(new_n296), .out0(new_n298));
  inv000aa1d42x5               g203(.a(new_n293), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[30] ), .b(\b[29] ), .c(new_n288), .carry(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n299), .c(new_n263), .d(new_n260), .o1(new_n301));
  aoi022aa1n03x5               g206(.a(new_n301), .b(new_n295), .c(new_n294), .d(new_n298), .o1(\s[31] ));
  xorb03aa1n02x5               g207(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nanp02aa1n02x5               g208(.a(new_n104), .b(new_n107), .o1(new_n304));
  nanp02aa1n02x5               g209(.a(new_n101), .b(new_n103), .o1(new_n305));
  oab012aa1n02x4               g210(.a(new_n102), .b(\a[3] ), .c(\b[2] ), .out0(new_n306));
  aoi022aa1n02x5               g211(.a(new_n304), .b(new_n105), .c(new_n306), .d(new_n305), .o1(\s[4] ));
  xnbna2aa1n03x5               g212(.a(new_n109), .b(new_n104), .c(new_n107), .out0(\s[5] ));
  norp02aa1n02x5               g213(.a(\b[4] ), .b(\a[5] ), .o1(new_n309));
  aoi012aa1n02x5               g214(.a(new_n309), .b(new_n304), .c(new_n109), .o1(new_n310));
  xnrb03aa1n02x5               g215(.a(new_n310), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g216(.a(new_n108), .b(new_n310), .out0(new_n312));
  xnbna2aa1n03x5               g217(.a(new_n113), .b(new_n312), .c(new_n115), .out0(\s[7] ));
  nanp02aa1n02x5               g218(.a(new_n312), .b(new_n116), .o1(new_n314));
  xnbna2aa1n03x5               g219(.a(new_n110), .b(new_n314), .c(new_n118), .out0(\s[8] ));
  xorb03aa1n02x5               g220(.a(new_n121), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


