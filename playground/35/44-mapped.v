// Benchmark "adder" written by ABC on Thu Jul 18 06:18:11 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n312, new_n313, new_n314, new_n317, new_n318, new_n320,
    new_n322, new_n323, new_n324;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  and002aa1n12x5               g002(.a(\b[1] ), .b(\a[2] ), .o(new_n98));
  nand22aa1n12x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nor042aa1n09x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  oab012aa1d15x5               g005(.a(new_n98), .b(new_n100), .c(new_n99), .out0(new_n101));
  nor042aa1n04x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nanp02aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  norb02aa1n06x5               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  nor002aa1n12x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nanp02aa1n24x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  norb02aa1d27x5               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  nand03aa1d16x5               g012(.a(new_n101), .b(new_n104), .c(new_n107), .o1(new_n108));
  aoi012aa1n09x5               g013(.a(new_n105), .b(new_n102), .c(new_n106), .o1(new_n109));
  nand22aa1n09x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nor042aa1n06x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nor042aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nand02aa1d06x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nano23aa1n06x5               g018(.a(new_n112), .b(new_n111), .c(new_n113), .d(new_n110), .out0(new_n114));
  xorc02aa1n12x5               g019(.a(\a[5] ), .b(\b[4] ), .out0(new_n115));
  nor022aa1n08x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nand02aa1d24x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  norb02aa1n06x4               g022(.a(new_n117), .b(new_n116), .out0(new_n118));
  nand23aa1n03x5               g023(.a(new_n114), .b(new_n115), .c(new_n118), .o1(new_n119));
  nano22aa1n03x7               g024(.a(new_n111), .b(new_n110), .c(new_n113), .out0(new_n120));
  oai022aa1n02x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  ao0012aa1n03x7               g026(.a(new_n116), .b(new_n111), .c(new_n117), .o(new_n122));
  aoi013aa1n06x4               g027(.a(new_n122), .b(new_n120), .c(new_n121), .d(new_n118), .o1(new_n123));
  aoai13aa1n12x5               g028(.a(new_n123), .b(new_n119), .c(new_n108), .d(new_n109), .o1(new_n124));
  tech160nm_fixorc02aa1n03p5x5 g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoi012aa1n02x5               g030(.a(new_n97), .b(new_n124), .c(new_n125), .o1(new_n126));
  nor042aa1d18x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand42aa1n08x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n03x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  norb03aa1n02x5               g034(.a(new_n128), .b(new_n97), .c(new_n127), .out0(new_n130));
  aob012aa1n06x5               g035(.a(new_n130), .b(new_n124), .c(new_n125), .out0(new_n131));
  oai012aa1n02x5               g036(.a(new_n131), .b(new_n126), .c(new_n129), .o1(\s[10] ));
  nand22aa1n04x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nor002aa1d32x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nano22aa1n02x4               g039(.a(new_n134), .b(new_n128), .c(new_n133), .out0(new_n135));
  inv030aa1n06x5               g040(.a(new_n134), .o1(new_n136));
  aoi022aa1n02x5               g041(.a(new_n131), .b(new_n128), .c(new_n136), .d(new_n133), .o1(new_n137));
  aoi012aa1n02x5               g042(.a(new_n137), .b(new_n131), .c(new_n135), .o1(\s[11] ));
  tech160nm_fiaoi012aa1n05x5   g043(.a(new_n134), .b(new_n131), .c(new_n135), .o1(new_n139));
  xnrb03aa1n03x5               g044(.a(new_n139), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor042aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand02aa1n08x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nona23aa1n09x5               g047(.a(new_n133), .b(new_n142), .c(new_n141), .d(new_n134), .out0(new_n143));
  nano22aa1n02x4               g048(.a(new_n143), .b(new_n125), .c(new_n129), .out0(new_n144));
  nanp02aa1n03x5               g049(.a(new_n124), .b(new_n144), .o1(new_n145));
  oai112aa1n06x5               g050(.a(new_n136), .b(new_n128), .c(new_n127), .d(new_n97), .o1(new_n146));
  aoi012aa1n06x5               g051(.a(new_n141), .b(new_n134), .c(new_n142), .o1(new_n147));
  oaih12aa1n12x5               g052(.a(new_n147), .b(new_n143), .c(new_n146), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  nanp02aa1n03x5               g054(.a(new_n145), .b(new_n149), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g056(.a(\a[13] ), .o1(new_n152));
  inv000aa1d42x5               g057(.a(\b[12] ), .o1(new_n153));
  oaoi03aa1n03x5               g058(.a(new_n152), .b(new_n153), .c(new_n150), .o1(new_n154));
  xnrb03aa1n03x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand42aa1d28x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nor002aa1d32x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nand42aa1d28x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nano23aa1d12x5               g064(.a(new_n156), .b(new_n158), .c(new_n159), .d(new_n157), .out0(new_n160));
  aoai13aa1n03x5               g065(.a(new_n160), .b(new_n148), .c(new_n124), .d(new_n144), .o1(new_n161));
  aoai13aa1n12x5               g066(.a(new_n159), .b(new_n158), .c(new_n152), .d(new_n153), .o1(new_n162));
  nor002aa1n04x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nand42aa1n06x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  norb02aa1n09x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n161), .c(new_n162), .out0(\s[15] ));
  nanp02aa1n03x5               g071(.a(new_n161), .b(new_n162), .o1(new_n167));
  xnrc02aa1n12x5               g072(.a(\b[15] ), .b(\a[16] ), .out0(new_n168));
  aoai13aa1n02x5               g073(.a(new_n168), .b(new_n163), .c(new_n167), .d(new_n164), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(new_n167), .b(new_n165), .o1(new_n170));
  nona22aa1n02x4               g075(.a(new_n170), .b(new_n168), .c(new_n163), .out0(new_n171));
  nanp02aa1n03x5               g076(.a(new_n171), .b(new_n169), .o1(\s[16] ));
  nano23aa1n06x5               g077(.a(new_n141), .b(new_n134), .c(new_n142), .d(new_n133), .out0(new_n173));
  nanb03aa1d18x5               g078(.a(new_n168), .b(new_n160), .c(new_n165), .out0(new_n174));
  nano32aa1d12x5               g079(.a(new_n174), .b(new_n173), .c(new_n129), .d(new_n125), .out0(new_n175));
  nanp02aa1n09x5               g080(.a(new_n124), .b(new_n175), .o1(new_n176));
  inv040aa1n03x5               g081(.a(new_n174), .o1(new_n177));
  norp02aa1n02x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanb02aa1n03x5               g083(.a(new_n163), .b(new_n162), .out0(new_n179));
  aoi022aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .c(\a[15] ), .d(\b[14] ), .o1(new_n180));
  tech160nm_fiaoi012aa1n02p5x5 g085(.a(new_n178), .b(new_n179), .c(new_n180), .o1(new_n181));
  aobi12aa1n12x5               g086(.a(new_n181), .b(new_n177), .c(new_n148), .out0(new_n182));
  xorc02aa1n02x5               g087(.a(\a[17] ), .b(\b[16] ), .out0(new_n183));
  xnbna2aa1n03x5               g088(.a(new_n183), .b(new_n176), .c(new_n182), .out0(\s[17] ));
  inv040aa1d32x5               g089(.a(\a[18] ), .o1(new_n185));
  nanp02aa1n06x5               g090(.a(new_n176), .b(new_n182), .o1(new_n186));
  norp02aa1n12x5               g091(.a(\b[16] ), .b(\a[17] ), .o1(new_n187));
  tech160nm_fiaoi012aa1n05x5   g092(.a(new_n187), .b(new_n186), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  nanb02aa1n02x5               g094(.a(new_n146), .b(new_n173), .out0(new_n190));
  aoai13aa1n06x5               g095(.a(new_n181), .b(new_n174), .c(new_n190), .d(new_n147), .o1(new_n191));
  inv000aa1d42x5               g096(.a(\a[17] ), .o1(new_n192));
  xroi22aa1d06x4               g097(.a(new_n192), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n193));
  aoai13aa1n06x5               g098(.a(new_n193), .b(new_n191), .c(new_n124), .d(new_n175), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\b[17] ), .o1(new_n195));
  oaoi03aa1n09x5               g100(.a(new_n185), .b(new_n195), .c(new_n187), .o1(new_n196));
  nor002aa1d24x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nanp02aa1n09x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  xnbna2aa1n03x5               g105(.a(new_n200), .b(new_n194), .c(new_n196), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n06x5               g107(.a(new_n194), .b(new_n196), .o1(new_n203));
  nor002aa1n12x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nand02aa1n10x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanb02aa1n02x5               g110(.a(new_n204), .b(new_n205), .out0(new_n206));
  aoai13aa1n03x5               g111(.a(new_n206), .b(new_n197), .c(new_n203), .d(new_n200), .o1(new_n207));
  nand22aa1n02x5               g112(.a(new_n203), .b(new_n200), .o1(new_n208));
  nona22aa1n03x5               g113(.a(new_n208), .b(new_n206), .c(new_n197), .out0(new_n209));
  nanp02aa1n02x5               g114(.a(new_n209), .b(new_n207), .o1(\s[20] ));
  nano23aa1n09x5               g115(.a(new_n197), .b(new_n204), .c(new_n205), .d(new_n198), .out0(new_n211));
  nanp02aa1n02x5               g116(.a(new_n193), .b(new_n211), .o1(new_n212));
  nona23aa1d18x5               g117(.a(new_n205), .b(new_n198), .c(new_n197), .d(new_n204), .out0(new_n213));
  aoi012aa1n12x5               g118(.a(new_n204), .b(new_n197), .c(new_n205), .o1(new_n214));
  oai012aa1d24x5               g119(.a(new_n214), .b(new_n213), .c(new_n196), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n212), .c(new_n176), .d(new_n182), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  xorc02aa1n02x5               g124(.a(\a[21] ), .b(\b[20] ), .out0(new_n220));
  xorc02aa1n02x5               g125(.a(\a[22] ), .b(\b[21] ), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  aoai13aa1n03x5               g127(.a(new_n222), .b(new_n219), .c(new_n217), .d(new_n220), .o1(new_n223));
  nand42aa1n02x5               g128(.a(new_n217), .b(new_n220), .o1(new_n224));
  nona22aa1n02x4               g129(.a(new_n224), .b(new_n222), .c(new_n219), .out0(new_n225));
  nanp02aa1n03x5               g130(.a(new_n225), .b(new_n223), .o1(\s[22] ));
  inv000aa1d42x5               g131(.a(\a[21] ), .o1(new_n227));
  inv040aa1d32x5               g132(.a(\a[22] ), .o1(new_n228));
  xroi22aa1d06x4               g133(.a(new_n227), .b(\b[20] ), .c(new_n228), .d(\b[21] ), .out0(new_n229));
  nand23aa1n04x5               g134(.a(new_n229), .b(new_n193), .c(new_n211), .o1(new_n230));
  inv000aa1d42x5               g135(.a(\b[21] ), .o1(new_n231));
  oao003aa1n02x5               g136(.a(new_n228), .b(new_n231), .c(new_n219), .carry(new_n232));
  aoi012aa1n02x5               g137(.a(new_n232), .b(new_n215), .c(new_n229), .o1(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n230), .c(new_n176), .d(new_n182), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  xorc02aa1n06x5               g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  tech160nm_fixnrc02aa1n05x5   g142(.a(\b[23] ), .b(\a[24] ), .out0(new_n238));
  aoai13aa1n03x5               g143(.a(new_n238), .b(new_n236), .c(new_n234), .d(new_n237), .o1(new_n239));
  nand42aa1n02x5               g144(.a(new_n234), .b(new_n237), .o1(new_n240));
  nona22aa1n03x5               g145(.a(new_n240), .b(new_n238), .c(new_n236), .out0(new_n241));
  nanp02aa1n03x5               g146(.a(new_n241), .b(new_n239), .o1(\s[24] ));
  norb02aa1n03x5               g147(.a(new_n237), .b(new_n238), .out0(new_n243));
  nano22aa1n02x4               g148(.a(new_n212), .b(new_n243), .c(new_n229), .out0(new_n244));
  aoai13aa1n03x5               g149(.a(new_n244), .b(new_n191), .c(new_n124), .d(new_n175), .o1(new_n245));
  oao003aa1n02x5               g150(.a(new_n185), .b(new_n195), .c(new_n187), .carry(new_n246));
  inv020aa1n03x5               g151(.a(new_n214), .o1(new_n247));
  aoai13aa1n06x5               g152(.a(new_n229), .b(new_n247), .c(new_n211), .d(new_n246), .o1(new_n248));
  inv000aa1n02x5               g153(.a(new_n232), .o1(new_n249));
  inv000aa1n02x5               g154(.a(new_n243), .o1(new_n250));
  aoi112aa1n02x5               g155(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n251));
  oab012aa1n02x4               g156(.a(new_n251), .b(\a[24] ), .c(\b[23] ), .out0(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n250), .c(new_n248), .d(new_n249), .o1(new_n253));
  nanb02aa1n03x5               g158(.a(new_n253), .b(new_n245), .out0(new_n254));
  xorb03aa1n02x5               g159(.a(new_n254), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  xorc02aa1n12x5               g161(.a(\a[25] ), .b(\b[24] ), .out0(new_n257));
  xorc02aa1n12x5               g162(.a(\a[26] ), .b(\b[25] ), .out0(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n259), .b(new_n256), .c(new_n254), .d(new_n257), .o1(new_n260));
  aoai13aa1n03x5               g165(.a(new_n257), .b(new_n253), .c(new_n186), .d(new_n244), .o1(new_n261));
  nona22aa1n03x5               g166(.a(new_n261), .b(new_n259), .c(new_n256), .out0(new_n262));
  nanp02aa1n03x5               g167(.a(new_n260), .b(new_n262), .o1(\s[26] ));
  and002aa1n24x5               g168(.a(new_n258), .b(new_n257), .o(new_n264));
  nano22aa1d15x5               g169(.a(new_n230), .b(new_n243), .c(new_n264), .out0(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n191), .c(new_n124), .d(new_n175), .o1(new_n266));
  aoi112aa1n02x5               g171(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n267));
  oab012aa1n02x4               g172(.a(new_n267), .b(\a[26] ), .c(\b[25] ), .out0(new_n268));
  aobi12aa1n06x5               g173(.a(new_n268), .b(new_n253), .c(new_n264), .out0(new_n269));
  xorc02aa1n12x5               g174(.a(\a[27] ), .b(\b[26] ), .out0(new_n270));
  xnbna2aa1n03x5               g175(.a(new_n270), .b(new_n269), .c(new_n266), .out0(\s[27] ));
  nanp02aa1n02x5               g176(.a(new_n253), .b(new_n264), .o1(new_n272));
  nanp03aa1n03x5               g177(.a(new_n272), .b(new_n266), .c(new_n268), .o1(new_n273));
  norp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  norp02aa1n02x5               g179(.a(\b[27] ), .b(\a[28] ), .o1(new_n275));
  nanp02aa1n02x5               g180(.a(\b[27] ), .b(\a[28] ), .o1(new_n276));
  nanb02aa1n02x5               g181(.a(new_n275), .b(new_n276), .out0(new_n277));
  aoai13aa1n03x5               g182(.a(new_n277), .b(new_n274), .c(new_n273), .d(new_n270), .o1(new_n278));
  aobi12aa1n12x5               g183(.a(new_n265), .b(new_n176), .c(new_n182), .out0(new_n279));
  aoai13aa1n02x7               g184(.a(new_n243), .b(new_n232), .c(new_n215), .d(new_n229), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n264), .o1(new_n281));
  aoai13aa1n04x5               g186(.a(new_n268), .b(new_n281), .c(new_n280), .d(new_n252), .o1(new_n282));
  oai012aa1n06x5               g187(.a(new_n270), .b(new_n282), .c(new_n279), .o1(new_n283));
  nona22aa1n02x4               g188(.a(new_n283), .b(new_n277), .c(new_n274), .out0(new_n284));
  nanp02aa1n03x5               g189(.a(new_n278), .b(new_n284), .o1(\s[28] ));
  norb02aa1n03x5               g190(.a(new_n270), .b(new_n277), .out0(new_n286));
  oaih12aa1n02x5               g191(.a(new_n286), .b(new_n282), .c(new_n279), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n286), .o1(new_n288));
  oai012aa1n02x5               g193(.a(new_n276), .b(new_n275), .c(new_n274), .o1(new_n289));
  aoai13aa1n02x7               g194(.a(new_n289), .b(new_n288), .c(new_n269), .d(new_n266), .o1(new_n290));
  xorc02aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .out0(new_n291));
  norb02aa1n02x5               g196(.a(new_n289), .b(new_n291), .out0(new_n292));
  aoi022aa1n03x5               g197(.a(new_n290), .b(new_n291), .c(new_n287), .d(new_n292), .o1(\s[29] ));
  xorb03aa1n02x5               g198(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanb03aa1n02x5               g199(.a(new_n277), .b(new_n291), .c(new_n270), .out0(new_n295));
  oabi12aa1n03x5               g200(.a(new_n295), .b(new_n282), .c(new_n279), .out0(new_n296));
  oao003aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n295), .c(new_n269), .d(new_n266), .o1(new_n298));
  xorc02aa1n02x5               g203(.a(\a[30] ), .b(\b[29] ), .out0(new_n299));
  norb02aa1n02x5               g204(.a(new_n297), .b(new_n299), .out0(new_n300));
  aoi022aa1n03x5               g205(.a(new_n298), .b(new_n299), .c(new_n296), .d(new_n300), .o1(\s[30] ));
  nand03aa1n02x5               g206(.a(new_n286), .b(new_n291), .c(new_n299), .o1(new_n302));
  oabi12aa1n03x5               g207(.a(new_n302), .b(new_n282), .c(new_n279), .out0(new_n303));
  xorc02aa1n02x5               g208(.a(\a[31] ), .b(\b[30] ), .out0(new_n304));
  and002aa1n02x5               g209(.a(\b[29] ), .b(\a[30] ), .o(new_n305));
  oabi12aa1n02x5               g210(.a(new_n304), .b(\a[30] ), .c(\b[29] ), .out0(new_n306));
  oab012aa1n02x4               g211(.a(new_n306), .b(new_n297), .c(new_n305), .out0(new_n307));
  oao003aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .c(new_n297), .carry(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n302), .c(new_n269), .d(new_n266), .o1(new_n309));
  aoi022aa1n03x5               g214(.a(new_n309), .b(new_n304), .c(new_n303), .d(new_n307), .o1(\s[31] ));
  xorb03aa1n02x5               g215(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  inv000aa1d42x5               g216(.a(new_n107), .o1(new_n312));
  nanp02aa1n02x5               g217(.a(new_n108), .b(new_n109), .o1(new_n313));
  oai112aa1n02x5               g218(.a(new_n103), .b(new_n312), .c(new_n101), .d(new_n102), .o1(new_n314));
  oai012aa1n02x5               g219(.a(new_n314), .b(new_n313), .c(new_n312), .o1(\s[4] ));
  xnbna2aa1n03x5               g220(.a(new_n115), .b(new_n108), .c(new_n109), .out0(\s[5] ));
  nanp03aa1n02x5               g221(.a(new_n108), .b(new_n109), .c(new_n115), .o1(new_n317));
  aob012aa1n02x5               g222(.a(new_n317), .b(\b[4] ), .c(\a[5] ), .out0(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g224(.a(new_n113), .b(new_n112), .c(new_n318), .out0(new_n320));
  xnrb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  norb02aa1n02x5               g226(.a(new_n320), .b(new_n111), .out0(new_n322));
  oaoi03aa1n02x5               g227(.a(\a[7] ), .b(\b[6] ), .c(new_n320), .o1(new_n323));
  oaib12aa1n02x5               g228(.a(new_n110), .b(new_n116), .c(new_n117), .out0(new_n324));
  obai22aa1n02x7               g229(.a(new_n118), .b(new_n323), .c(new_n322), .d(new_n324), .out0(\s[8] ));
  xorb03aa1n02x5               g230(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


