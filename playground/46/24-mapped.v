// Benchmark "adder" written by ABC on Thu Jul 18 11:46:14 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n317, new_n319, new_n320, new_n321, new_n322, new_n325, new_n327,
    new_n328, new_n330;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand02aa1n06x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n12x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  inv040aa1d32x5               g004(.a(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[8] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nor002aa1d32x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand42aa1d28x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norb02aa1n03x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  nand22aa1n12x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  nand42aa1n20x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nor042aa1n09x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nona22aa1n09x5               g013(.a(new_n107), .b(new_n108), .c(new_n106), .out0(new_n109));
  nand42aa1d28x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nor042aa1n04x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nano22aa1n09x5               g016(.a(new_n111), .b(new_n107), .c(new_n110), .out0(new_n112));
  nand03aa1n06x5               g017(.a(new_n112), .b(new_n109), .c(new_n105), .o1(new_n113));
  aoi012aa1n12x5               g018(.a(new_n103), .b(new_n111), .c(new_n104), .o1(new_n114));
  xorc02aa1n12x5               g019(.a(\a[6] ), .b(\b[5] ), .out0(new_n115));
  xorc02aa1n12x5               g020(.a(\a[5] ), .b(\b[4] ), .out0(new_n116));
  nor042aa1n06x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nand02aa1d24x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nor042aa1n06x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nanp02aa1n24x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nano23aa1n09x5               g025(.a(new_n117), .b(new_n119), .c(new_n120), .d(new_n118), .out0(new_n121));
  nand23aa1n04x5               g026(.a(new_n121), .b(new_n115), .c(new_n116), .o1(new_n122));
  ao0012aa1n03x7               g027(.a(new_n117), .b(new_n119), .c(new_n118), .o(new_n123));
  orn002aa1n24x5               g028(.a(\a[5] ), .b(\b[4] ), .o(new_n124));
  oaoi03aa1n09x5               g029(.a(\a[6] ), .b(\b[5] ), .c(new_n124), .o1(new_n125));
  aoi012aa1n06x5               g030(.a(new_n123), .b(new_n121), .c(new_n125), .o1(new_n126));
  aoai13aa1n12x5               g031(.a(new_n126), .b(new_n122), .c(new_n113), .d(new_n114), .o1(new_n127));
  xnrc02aa1n12x5               g032(.a(\b[8] ), .b(\a[9] ), .out0(new_n128));
  nanb02aa1n06x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  xobna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n102), .out0(\s[10] ));
  aoai13aa1n06x5               g035(.a(new_n98), .b(new_n97), .c(new_n100), .d(new_n101), .o1(new_n131));
  aoai13aa1n06x5               g036(.a(new_n131), .b(new_n99), .c(new_n129), .d(new_n102), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n24x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1n20x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  nor002aa1d32x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand02aa1d28x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(new_n140));
  aoai13aa1n03x5               g045(.a(new_n140), .b(new_n134), .c(new_n132), .d(new_n137), .o1(new_n141));
  nand02aa1n02x5               g046(.a(new_n132), .b(new_n137), .o1(new_n142));
  nona22aa1n02x4               g047(.a(new_n142), .b(new_n140), .c(new_n134), .out0(new_n143));
  nanp02aa1n03x5               g048(.a(new_n143), .b(new_n141), .o1(\s[12] ));
  nano23aa1d15x5               g049(.a(new_n134), .b(new_n138), .c(new_n139), .d(new_n135), .out0(new_n145));
  nona22aa1d18x5               g050(.a(new_n145), .b(new_n128), .c(new_n99), .out0(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  nanp02aa1n03x5               g052(.a(new_n127), .b(new_n147), .o1(new_n148));
  nona23aa1n09x5               g053(.a(new_n139), .b(new_n135), .c(new_n134), .d(new_n138), .out0(new_n149));
  ao0012aa1n12x5               g054(.a(new_n138), .b(new_n134), .c(new_n139), .o(new_n150));
  oabi12aa1n18x5               g055(.a(new_n150), .b(new_n149), .c(new_n131), .out0(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  nanp02aa1n03x5               g057(.a(new_n148), .b(new_n152), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n04x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand42aa1n04x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  tech160nm_fiaoi012aa1n05x5   g061(.a(new_n155), .b(new_n153), .c(new_n156), .o1(new_n157));
  xnrb03aa1n03x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n04x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nand42aa1n08x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nano23aa1n09x5               g065(.a(new_n155), .b(new_n159), .c(new_n160), .d(new_n156), .out0(new_n161));
  aoai13aa1n06x5               g066(.a(new_n161), .b(new_n151), .c(new_n127), .d(new_n147), .o1(new_n162));
  aoi012aa1d18x5               g067(.a(new_n159), .b(new_n155), .c(new_n160), .o1(new_n163));
  nor042aa1n04x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nand42aa1n06x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nanb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n162), .c(new_n163), .out0(\s[15] ));
  nanp02aa1n03x5               g073(.a(new_n162), .b(new_n163), .o1(new_n169));
  nor002aa1n06x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand02aa1n12x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  aoai13aa1n02x7               g077(.a(new_n172), .b(new_n164), .c(new_n169), .d(new_n167), .o1(new_n173));
  aoi112aa1n03x5               g078(.a(new_n164), .b(new_n172), .c(new_n169), .d(new_n165), .o1(new_n174));
  nanb02aa1n03x5               g079(.a(new_n174), .b(new_n173), .out0(\s[16] ));
  inv000aa1d42x5               g080(.a(\a[17] ), .o1(new_n176));
  nano23aa1n06x5               g081(.a(new_n164), .b(new_n170), .c(new_n171), .d(new_n165), .out0(new_n177));
  nano22aa1d15x5               g082(.a(new_n146), .b(new_n161), .c(new_n177), .out0(new_n178));
  inv030aa1n02x5               g083(.a(new_n177), .o1(new_n179));
  oaoi03aa1n02x5               g084(.a(\a[10] ), .b(\b[9] ), .c(new_n102), .o1(new_n180));
  aoai13aa1n04x5               g085(.a(new_n161), .b(new_n150), .c(new_n145), .d(new_n180), .o1(new_n181));
  aoi012aa1n09x5               g086(.a(new_n179), .b(new_n181), .c(new_n163), .o1(new_n182));
  aoi012aa1d24x5               g087(.a(new_n170), .b(new_n164), .c(new_n171), .o1(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  aoi112aa1n09x5               g089(.a(new_n182), .b(new_n184), .c(new_n127), .d(new_n178), .o1(new_n185));
  xorb03aa1n03x5               g090(.a(new_n185), .b(\b[16] ), .c(new_n176), .out0(\s[17] ));
  nanb02aa1n03x5               g091(.a(\b[16] ), .b(new_n176), .out0(new_n187));
  aoai13aa1n06x5               g092(.a(new_n183), .b(new_n179), .c(new_n181), .d(new_n163), .o1(new_n188));
  xorc02aa1n12x5               g093(.a(\a[17] ), .b(\b[16] ), .out0(new_n189));
  aoai13aa1n03x5               g094(.a(new_n189), .b(new_n188), .c(new_n127), .d(new_n178), .o1(new_n190));
  xnrc02aa1n02x5               g095(.a(\b[17] ), .b(\a[18] ), .out0(new_n191));
  xobna2aa1n03x5               g096(.a(new_n191), .b(new_n190), .c(new_n187), .out0(\s[18] ));
  inv000aa1d42x5               g097(.a(\a[18] ), .o1(new_n193));
  xroi22aa1d06x4               g098(.a(new_n176), .b(\b[16] ), .c(new_n193), .d(\b[17] ), .out0(new_n194));
  inv000aa1n02x5               g099(.a(new_n194), .o1(new_n195));
  oaih22aa1d12x5               g100(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n196));
  oaib12aa1n18x5               g101(.a(new_n196), .b(new_n193), .c(\b[17] ), .out0(new_n197));
  tech160nm_fioai012aa1n05x5   g102(.a(new_n197), .b(new_n185), .c(new_n195), .o1(new_n198));
  xorb03aa1n02x5               g103(.a(new_n198), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nand42aa1d28x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanb02aa1d24x5               g107(.a(new_n201), .b(new_n202), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  nor002aa1d24x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nand42aa1d28x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanb02aa1n12x5               g111(.a(new_n205), .b(new_n206), .out0(new_n207));
  aoai13aa1n03x5               g112(.a(new_n207), .b(new_n201), .c(new_n198), .d(new_n204), .o1(new_n208));
  tech160nm_finand02aa1n05x5   g113(.a(new_n127), .b(new_n178), .o1(new_n209));
  inv000aa1d42x5               g114(.a(new_n163), .o1(new_n210));
  aoai13aa1n06x5               g115(.a(new_n177), .b(new_n210), .c(new_n151), .d(new_n161), .o1(new_n211));
  nand23aa1n06x5               g116(.a(new_n209), .b(new_n211), .c(new_n183), .o1(new_n212));
  oaoi03aa1n12x5               g117(.a(\a[18] ), .b(\b[17] ), .c(new_n187), .o1(new_n213));
  aoai13aa1n03x5               g118(.a(new_n204), .b(new_n213), .c(new_n212), .d(new_n194), .o1(new_n214));
  nona22aa1n02x4               g119(.a(new_n214), .b(new_n207), .c(new_n201), .out0(new_n215));
  nanp02aa1n03x5               g120(.a(new_n208), .b(new_n215), .o1(\s[20] ));
  nano23aa1d15x5               g121(.a(new_n201), .b(new_n205), .c(new_n206), .d(new_n202), .out0(new_n217));
  nanb03aa1d24x5               g122(.a(new_n191), .b(new_n217), .c(new_n189), .out0(new_n218));
  tech160nm_fiaoi012aa1n05x5   g123(.a(new_n205), .b(new_n201), .c(new_n206), .o1(new_n219));
  oai013aa1d12x5               g124(.a(new_n219), .b(new_n197), .c(new_n203), .d(new_n207), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  tech160nm_fioai012aa1n05x5   g126(.a(new_n221), .b(new_n185), .c(new_n218), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n04x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  xnrc02aa1n12x5               g129(.a(\b[20] ), .b(\a[21] ), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  xnrc02aa1n12x5               g131(.a(\b[21] ), .b(\a[22] ), .out0(new_n227));
  aoai13aa1n03x5               g132(.a(new_n227), .b(new_n224), .c(new_n222), .d(new_n226), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n218), .o1(new_n229));
  aoai13aa1n03x5               g134(.a(new_n226), .b(new_n220), .c(new_n212), .d(new_n229), .o1(new_n230));
  nona22aa1n02x4               g135(.a(new_n230), .b(new_n227), .c(new_n224), .out0(new_n231));
  nanp02aa1n03x5               g136(.a(new_n228), .b(new_n231), .o1(\s[22] ));
  nor042aa1n06x5               g137(.a(new_n227), .b(new_n225), .o1(new_n233));
  nano22aa1n03x7               g138(.a(new_n195), .b(new_n233), .c(new_n217), .out0(new_n234));
  inv000aa1n02x5               g139(.a(new_n234), .o1(new_n235));
  inv000aa1d42x5               g140(.a(\a[22] ), .o1(new_n236));
  inv000aa1d42x5               g141(.a(\b[21] ), .o1(new_n237));
  oaoi03aa1n09x5               g142(.a(new_n236), .b(new_n237), .c(new_n224), .o1(new_n238));
  inv000aa1n02x5               g143(.a(new_n238), .o1(new_n239));
  tech160nm_fiaoi012aa1n05x5   g144(.a(new_n239), .b(new_n220), .c(new_n233), .o1(new_n240));
  tech160nm_fioai012aa1n05x5   g145(.a(new_n240), .b(new_n185), .c(new_n235), .o1(new_n241));
  xorb03aa1n02x5               g146(.a(new_n241), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  xorc02aa1n12x5               g148(.a(\a[23] ), .b(\b[22] ), .out0(new_n244));
  tech160nm_fixnrc02aa1n05x5   g149(.a(\b[23] ), .b(\a[24] ), .out0(new_n245));
  aoai13aa1n03x5               g150(.a(new_n245), .b(new_n243), .c(new_n241), .d(new_n244), .o1(new_n246));
  inv000aa1n02x5               g151(.a(new_n240), .o1(new_n247));
  aoai13aa1n03x5               g152(.a(new_n244), .b(new_n247), .c(new_n212), .d(new_n234), .o1(new_n248));
  nona22aa1n02x4               g153(.a(new_n248), .b(new_n245), .c(new_n243), .out0(new_n249));
  nanp02aa1n03x5               g154(.a(new_n246), .b(new_n249), .o1(\s[24] ));
  norb02aa1n03x4               g155(.a(new_n244), .b(new_n245), .out0(new_n251));
  inv040aa1n02x5               g156(.a(new_n251), .o1(new_n252));
  nano32aa1n03x7               g157(.a(new_n252), .b(new_n194), .c(new_n233), .d(new_n217), .out0(new_n253));
  inv000aa1n02x5               g158(.a(new_n253), .o1(new_n254));
  inv000aa1n02x5               g159(.a(new_n219), .o1(new_n255));
  aoai13aa1n06x5               g160(.a(new_n233), .b(new_n255), .c(new_n217), .d(new_n213), .o1(new_n256));
  orn002aa1n02x5               g161(.a(\a[23] ), .b(\b[22] ), .o(new_n257));
  oao003aa1n02x5               g162(.a(\a[24] ), .b(\b[23] ), .c(new_n257), .carry(new_n258));
  aoai13aa1n06x5               g163(.a(new_n258), .b(new_n252), .c(new_n256), .d(new_n238), .o1(new_n259));
  inv000aa1n02x5               g164(.a(new_n259), .o1(new_n260));
  tech160nm_fioai012aa1n05x5   g165(.a(new_n260), .b(new_n185), .c(new_n254), .o1(new_n261));
  xorb03aa1n02x5               g166(.a(new_n261), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  xorc02aa1n12x5               g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  xnrc02aa1n12x5               g169(.a(\b[25] ), .b(\a[26] ), .out0(new_n265));
  aoai13aa1n03x5               g170(.a(new_n265), .b(new_n263), .c(new_n261), .d(new_n264), .o1(new_n266));
  aoai13aa1n03x5               g171(.a(new_n264), .b(new_n259), .c(new_n212), .d(new_n253), .o1(new_n267));
  nona22aa1n02x4               g172(.a(new_n267), .b(new_n265), .c(new_n263), .out0(new_n268));
  nanp02aa1n03x5               g173(.a(new_n266), .b(new_n268), .o1(\s[26] ));
  norb02aa1n06x5               g174(.a(new_n264), .b(new_n265), .out0(new_n270));
  nano23aa1d15x5               g175(.a(new_n218), .b(new_n252), .c(new_n270), .d(new_n233), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n188), .c(new_n127), .d(new_n178), .o1(new_n272));
  inv040aa1d32x5               g177(.a(\a[26] ), .o1(new_n273));
  inv000aa1d42x5               g178(.a(\b[25] ), .o1(new_n274));
  oaoi03aa1n06x5               g179(.a(new_n273), .b(new_n274), .c(new_n263), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  aoi012aa1n06x5               g181(.a(new_n276), .b(new_n259), .c(new_n270), .o1(new_n277));
  xorc02aa1n12x5               g182(.a(\a[27] ), .b(\b[26] ), .out0(new_n278));
  xnbna2aa1n03x5               g183(.a(new_n278), .b(new_n272), .c(new_n277), .out0(\s[27] ));
  inv000aa1n02x5               g184(.a(new_n271), .o1(new_n280));
  tech160nm_fioai012aa1n05x5   g185(.a(new_n277), .b(new_n185), .c(new_n280), .o1(new_n281));
  nor042aa1n03x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  norp02aa1n09x5               g187(.a(\b[27] ), .b(\a[28] ), .o1(new_n283));
  nand02aa1n04x5               g188(.a(\b[27] ), .b(\a[28] ), .o1(new_n284));
  nanb02aa1n12x5               g189(.a(new_n283), .b(new_n284), .out0(new_n285));
  aoai13aa1n03x5               g190(.a(new_n285), .b(new_n282), .c(new_n281), .d(new_n278), .o1(new_n286));
  aoai13aa1n03x5               g191(.a(new_n251), .b(new_n239), .c(new_n220), .d(new_n233), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n270), .o1(new_n288));
  aoai13aa1n04x5               g193(.a(new_n275), .b(new_n288), .c(new_n287), .d(new_n258), .o1(new_n289));
  aoai13aa1n02x5               g194(.a(new_n278), .b(new_n289), .c(new_n212), .d(new_n271), .o1(new_n290));
  nona22aa1n03x5               g195(.a(new_n290), .b(new_n285), .c(new_n282), .out0(new_n291));
  nanp02aa1n03x5               g196(.a(new_n286), .b(new_n291), .o1(\s[28] ));
  norb02aa1n03x5               g197(.a(new_n278), .b(new_n285), .out0(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n289), .c(new_n212), .d(new_n271), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n293), .o1(new_n295));
  oai012aa1n02x5               g200(.a(new_n284), .b(new_n283), .c(new_n282), .o1(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n295), .c(new_n272), .d(new_n277), .o1(new_n297));
  xorc02aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .out0(new_n298));
  norb02aa1n02x5               g203(.a(new_n296), .b(new_n298), .out0(new_n299));
  aoi022aa1n03x5               g204(.a(new_n297), .b(new_n298), .c(new_n294), .d(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g206(.a(new_n285), .b(new_n278), .c(new_n298), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n289), .c(new_n212), .d(new_n271), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n302), .o1(new_n304));
  oao003aa1n03x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n305));
  aoai13aa1n02x7               g210(.a(new_n305), .b(new_n304), .c(new_n272), .d(new_n277), .o1(new_n306));
  xorc02aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .out0(new_n307));
  norb02aa1n02x5               g212(.a(new_n305), .b(new_n307), .out0(new_n308));
  aoi022aa1n02x7               g213(.a(new_n306), .b(new_n307), .c(new_n303), .d(new_n308), .o1(\s[30] ));
  nand23aa1n03x5               g214(.a(new_n293), .b(new_n298), .c(new_n307), .o1(new_n310));
  nanb02aa1n03x5               g215(.a(new_n310), .b(new_n281), .out0(new_n311));
  xorc02aa1n02x5               g216(.a(\a[31] ), .b(\b[30] ), .out0(new_n312));
  oao003aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n313));
  norb02aa1n02x5               g218(.a(new_n313), .b(new_n312), .out0(new_n314));
  aoai13aa1n02x7               g219(.a(new_n313), .b(new_n310), .c(new_n272), .d(new_n277), .o1(new_n315));
  aoi022aa1n03x5               g220(.a(new_n315), .b(new_n312), .c(new_n311), .d(new_n314), .o1(\s[31] ));
  norb02aa1n02x5               g221(.a(new_n110), .b(new_n111), .out0(new_n317));
  xobna2aa1n03x5               g222(.a(new_n317), .b(new_n109), .c(new_n107), .out0(\s[3] ));
  inv000aa1d42x5               g223(.a(new_n103), .o1(new_n319));
  oai112aa1n02x5               g224(.a(new_n317), .b(new_n107), .c(new_n108), .d(new_n106), .o1(new_n320));
  nanp02aa1n02x5               g225(.a(new_n113), .b(new_n114), .o1(new_n321));
  aoi012aa1n02x5               g226(.a(new_n111), .b(new_n319), .c(new_n104), .o1(new_n322));
  aoi022aa1n02x5               g227(.a(new_n321), .b(new_n319), .c(new_n320), .d(new_n322), .o1(\s[4] ));
  xnbna2aa1n03x5               g228(.a(new_n116), .b(new_n113), .c(new_n114), .out0(\s[5] ));
  nanp02aa1n02x5               g229(.a(new_n321), .b(new_n116), .o1(new_n325));
  xnbna2aa1n03x5               g230(.a(new_n115), .b(new_n325), .c(new_n124), .out0(\s[6] ));
  and002aa1n02x5               g231(.a(new_n116), .b(new_n115), .o(new_n327));
  tech160nm_fiao0012aa1n02p5x5 g232(.a(new_n125), .b(new_n321), .c(new_n327), .o(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g234(.a(new_n119), .b(new_n328), .c(new_n120), .o1(new_n330));
  xnrb03aa1n02x5               g235(.a(new_n330), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g236(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


